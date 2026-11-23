'use strict';
// Actual independent launcher + Chrome + ROS + exact native C. Localhost only.
const fs=require('fs'),path=require('path'),assert=require('assert'),{spawn,spawnSync}=require('child_process');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=__dirname,name=process.argv[2],folder=path.resolve(process.argv[3]);
assert(['no_start','timer60','restart'].includes(name));fs.mkdirSync(folder,{recursive:true});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
async function until(fn,ms=30000){const t=Date.now();while(!await fn()){if(Date.now()-t>ms)throw Error('timeout');await sleep(25);}}
function traces(session){const text=fs.readFileSync(path.join(session,'TRACE.jsonl'),'utf8');return text.slice(0,text.lastIndexOf('\n')).trim().split('\n').filter(Boolean).map(JSON.parse);}
async function main(){
 const p=spawn('/usr/bin/python3',['-B',path.join(root,'entry.py'),'--rehearsal','--seconds',name==='timer60'?'60':'20','--no-browser'],{env:{...process.env,PYTHONDONTWRITEBYTECODE:'1'}});
 let stdout='',stderr='',browser,page,info,report,client,frozen=false;
 const errors=[];p.stdout.on('data',b=>stdout+=b);p.stderr.on('data',b=>stderr+=b);
 try{
  await until(()=>stdout.includes('BROWSER_READY '),50000);
  info=JSON.parse(stdout.split('BROWSER_READY ')[1].split('\n')[0]);
  assert(!info.hardware_entry&&info.synthetic&&info.automatic_motion===false);
  // No retry/wait workaround: CURRENT must already be published at READY.
  assert.equal(JSON.parse(fs.readFileSync(path.join(root,'CURRENT.json'))).url,info.url);
  assert.equal(fs.statSync(path.join(root,'CURRENT.json')).mode&0o777,0o600);
  const profile=JSON.parse(fs.readFileSync(path.join(info.startup_folder,'PROFILE.json')));
  assert.equal(profile.hardware.esp_ip,'127.0.0.1');assert.equal(profile.hardware.network_interface,'lo');
  assert(![46000,46001].includes(profile.hardware.command_port));assert(![46000,46001].includes(profile.hardware.telemetry_port));
  assert(!fs.existsSync(path.join(info.startup_folder,'RECEIVER.json')),'synthetic test must not start/read real Vicon');
  browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--disable-gpu'],
   ignoreDefaultArgs:['--disable-background-timer-throttling','--disable-backgrounding-occluded-windows','--disable-renderer-backgrounding']});
  const context=await browser.newContext({viewport:{width:1280,height:720}});page=await context.newPage();
  page.on('pageerror',e=>errors.push(String(e)));await page.goto(info.url);await page.bringToFront();
  if(name==='no_start'){
   // A key held before readiness must not become a delayed start on release.
   await page.keyboard.down('Space');
  }
  await until(()=>page.evaluate(()=>latest?.prepare_phase==='READY'&&!!offer),25000);
  const resultPath=path.join(info.session,'result.json');
  if(name==='no_start'){
   await page.keyboard.up('Space');await sleep(400);
   await page.keyboard.up('Space'); // Release without a fresh press.
   await page.evaluate(()=>{
    document.dispatchEvent(new KeyboardEvent('keydown',{code:'Space',bubbles:true}));
    document.dispatchEvent(new KeyboardEvent('keyup',{code:'Space',bubbles:true}));
   });
   await page.keyboard.down('Space');await sleep(950);await page.keyboard.up('Space');
   await sleep(1000);
   assert(!traces(info.session).some(v=>v.status.state==='ACTIVE'),'implicit/stale/untrusted input acquired authority');
   const stop=spawnSync('/usr/bin/python3',['-B',path.join(root,'entry.py'),'--stop'],{encoding:'utf8',timeout:6000});
   assert.equal(stop.status,0,stop.stderr);
  }else{
   await page.keyboard.press('Space');await until(()=>traces(info.session).at(-1)?.status.state==='ACTIVE',8000);
   if(name==='timer60'){
    // Rendering and foreground are not a control heartbeat. Expiry must save
    // while rendering is frozen, with unchanged100ms backend protection.
    await sleep(1500);const other=await context.newPage();await other.goto('about:blank');await other.bringToFront();
    client=await context.newCDPSession(page);await client.send('Page.setWebLifecycleState',{state:'frozen'});frozen=true;
    await until(()=>fs.existsSync(resultPath),65000);
    await client.send('Page.setWebLifecycleState',{state:'active'});frozen=false;
    await page.bringToFront();
   }else{
    await sleep(1000);await page.keyboard.press('Space');
   }
  }
  await until(()=>fs.existsSync(resultPath),10000);
  const r=JSON.parse(fs.readFileSync(resultPath));assert.equal(r.ros_domain,198);
  assert(!r.hardware_socket_opened&&!r.physical_motion_started&&r.stop_evidence.gate_output_zero&&r.stop_evidence.stop_confirmed);
  assert.equal(r.accepted_commands,name==='no_start'?0:1);
  const peer=r.stop_evidence.exact_C_peer;assert(peer&&peer.final_sample);
  assert(peer.final_sample.status.raw_duty.every(v=>v===0)&&peer.final_sample.status.target_cps.every(v=>v===0));
  let activeS=0;
  if(name==='no_start'){
   assert.equal(r.reason,'LOCAL_OPERATOR_STOP');assert(!r.authority_ever_acquired&&!peer.permit_ever&&!peer.peak_duty&&!peer.peak_rpm);
  }else if(name==='timer60'){
   assert(r.completed&&r.passed&&r.reason==='TIMER_COMPLETE');
   const transitions=r.stop_evidence.transitions;
   activeS=transitions.find(v=>v.state==='STOPPING').t-transitions.find(v=>v.state==='ACTIVE').t;
   assert(activeS>=60&&activeS<=60.1,'60s backend expiry outside unchanged100ms window');
  }else assert.equal(r.reason,'CLIENT_SECOND_SPACE');
  await until(()=>page.evaluate(()=>resultSeen&&latest.state==='STOPPED_CONFIRMED'),5000);
  if(name==='restart'){
   await page.keyboard.press('Space');await sleep(300);
   assert.equal(await page.evaluate(()=>latest.state),'STOPPED_CONFIRMED','terminal page must not restart');
  }
  await page.screenshot({path:path.join(folder,'page.png')});
  await until(()=>p.exitCode!==null,10000);assert.equal(p.exitCode,0,stderr);assert.deepEqual(errors,[]);
  report={passed:true,case:name,actual_public_launcher:true,actual_browser:true,actual_ROS_native_C_loopback:true,
   hardware_socket_opened:false,real_Vicon_started:false,physical_output:false,parent_sources_unchanged:true,
   accepted_commands:r.accepted_commands,stop_confirmed:true,native_output_zero:true,active_s:activeS,
   frozen_renderer_timer:name==='timer60',metadata_ready_before_stdout:true,session:info.session,
   reason:r.reason,errors};
 }catch(e){report={passed:false,case:name,error:e.stack,info,errors};throw e;}
 finally{
  if(frozen&&client)try{await client.send('Page.setWebLifecycleState',{state:'active'});}catch{}
  if(browser)await browser.close();
  if(p.exitCode===null){p.kill('SIGTERM');await sleep(1000);}
  fs.writeFileSync(path.join(folder,'stdout.log'),stdout);fs.writeFileSync(path.join(folder,'stderr.log'),stderr);
  fs.writeFileSync(path.join(folder,'RESULT.json'),JSON.stringify(report,null,2)+'\n');
 }
 console.log(JSON.stringify(report));
}
main().catch(e=>{console.error(e);process.exitCode=1;});
