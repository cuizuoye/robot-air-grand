'use strict';
// Actual headless Chromium rendering; all HTTP and WebSocket data are mocked.
// No ROS nodes, real server, car ports, shared operator locks or motion IO.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=__dirname,web=path.join(root,'host/web');
async function main(){
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--disable-gpu']});
 const results=[],errors=[];
 try{
  for(const fixture of [
   {name:'car_timeout',code:'CAR_POSE_TIMEOUT',title:'小车定位更新超时',age:.23455},
   {name:'wand_timeout',code:'WAND_POSE_TIMEOUT',title:'标定杆定位更新超时',age:.214434},
   {name:'prepare_preserves_cause',code:'PREPARE_BLOCKED: 状态已撤销或存在故障',cause:'WAND_POSE_TIMEOUT',title:'标定杆定位更新超时',age:.200504},
   {name:'unknown_exception',code:'RuntimeError: private_function_name()',title:'未分类的软件异常',age:.2},
   {name:'unconfirmed_stop',code:'CAR_POSE_TIMEOUT',title:'小车定位更新超时',age:.23,unconfirmed:true},
  ]){
   const context=await browser.newContext({viewport:{width:1280,height:720}});
   await context.addInitScript(()=>{
    globalThis.fakeSockets=[];
    class FakeSocket{
     static OPEN=1;
     constructor(url){this.url=url;this.readyState=1;this.sent=[];fakeSockets.push(this);setTimeout(()=>this.onopen?.(),0);}
     send(value){this.sent.push(JSON.parse(value));}
    }
    globalThis.WebSocket=FakeSocket;
    globalThis.fakeEmit=(role,value)=>fakeSockets.find(s=>s.url.endsWith('/'+role)).onmessage({data:JSON.stringify(value)});
   });
   await context.route('**/*',async route=>{
    const name=new URL(route.request().url()).pathname;
    const file=name==='/'?'index.html':name.slice(1);
    if(!/^[a-z_]+\.(html|js|css)$/.test(file))return route.abort();
    const type=file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/html';
    await route.fulfill({status:200,contentType:type+'; charset=utf-8',body:fs.readFileSync(path.join(web,file))});
   });
   const page=await context.newPage();page.on('pageerror',e=>errors.push(String(e)));
   await page.goto('http://127.0.0.1:39871/#'+'a'.repeat(48));await page.bringToFront();
   await page.waitForFunction(()=>fakeSockets.some(s=>s.url.endsWith('/control')),{},{timeout:15000});
   await page.evaluate(()=>fakeEmit('control',{kind:'hello',mode:'hardware',owner:'offline-fixture',authority_mode:'WAND_BACKEND_SESSION_V1'}));
   await page.waitForFunction(()=>fakeSockets.some(s=>s.url.endsWith('/view')));
   const diag={pose_source_ages_s:{car:fixture.age,wand:fixture.age},native_fault_bits:0};
   await page.evaluate(({fixture,diag})=>fakeEmit('view',{
    kind:'view',view_sequence:1,state:'FAULT',reason:fixture.cause||fixture.code,
    cause_reason:fixture.cause||fixture.code,channel:{fault:fixture.code},display_diagnostics:diag,
    prepare_phase:'BLOCKED',remaining_s:600,car:{x:1,y:-1,yaw:0},trace:[],
   }),{fixture,diag});
   await page.evaluate(({fixture,diag})=>fakeEmit('control',{kind:'result',result:{
    reason:fixture.code,cause_reason:fixture.cause||fixture.code,display_diagnostics:diag,
    stop_evidence:{stop_confirmed:!fixture.unconfirmed,gate_output_zero:true},
   }}),{fixture,diag});
   await page.waitForFunction(()=>resultSeen);
   assert.equal(await page.locator('#error-title').innerText(),fixture.title);
   assert.equal(await page.locator('#error-panel').isVisible(),true);
   assert((await page.locator('#error-facts').innerText()).includes((fixture.age*1000).toFixed(1)+'毫秒'));
   if(fixture.unconfirmed)assert((await page.locator('#error-action').innerText()).includes('立即查看实车'));
   // Terminal-page state updates cannot overwrite the original displayed cause.
   await page.waitForTimeout(180);
   const text=await page.locator('body').innerText();
   assert(!/CAR_POSE_TIMEOUT|WAND_POSE_TIMEOUT|PREPARE_BLOCKED|RuntimeError|private_function_name|STOPPED_CONFIRMED|STOP_UNCONFIRMED/.test(text),text);
   const messages=await page.evaluate(()=>fakeSockets.flatMap(s=>s.sent));
   assert(!messages.some(m=>m.type==='command'),'fault display must never issue START');
   results.push({name:fixture.name,passed:true});
   await context.close();
  }
  assert.deepEqual(errors,[]);
 }finally{await browser.close();}
 console.log(JSON.stringify({passed:true,cases:results,errors,hardware_socket_opened:false,ros_initialized:false,shared_operator_lock_used:false}));
}
main().catch(e=>{console.error(e);process.exitCode=1;});
