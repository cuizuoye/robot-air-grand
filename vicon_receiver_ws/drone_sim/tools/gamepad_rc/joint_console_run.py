"""GUI adapter for the EXISTING terminal owner. No device, ROS or control loop.

Opening the GUI is inert. Explicit clicks launch one child and send the same
ground/review/CAR_START/preview/TAKEOFF/cancel lines as the terminal workflow.
Child stdout goes to disk, so a slow/closed GUI cannot block its flight loop.
"""
from datetime import datetime
import json
from pathlib import Path
import shlex
import subprocess
import time

TOOLS=Path(__file__).resolve().parent
RUNS=TOOLS.parents[1]/'preparation_runs'
PROFILE=TOOLS/'plan_b/fixed_point/vicon_profile.initial_trial.json'
PLANS={'矩形 3×2 m':TOOLS/'plan_b/follow/air_ground_rectangle.candidate.json',
       '圆周 直径1.5 m':TOOLS/'plan_b/follow/air_ground_circle.candidate.json',
       '短路线 1 m 往返':TOOLS/'plan_b/follow/air_ground_side_follow.current.json'}


def route_description(plan):
    circle=plan.get('circle_route')
    if circle:
        return (f"直径 {circle['diameter_m']:g} m，圆心在起点左侧 {circle['diameter_m']/2:g} m；"
                '先向前，固定车头逆时针一圈，回到本轮小车起点停车')
    return f"前/左坐标（m）：{plan['route_local_forward_left_m']}"

# Verified saved profiles; never select an adapter by enumeration order.
DEDICATED_WIFI=(
    ('小车','wlx502b73a508c9','50:2B:73:A5:08:C9','79614b9e-7213-452a-be72-34f9cdf1e9aa',
     'Mecanum-Bench-S3','192.168.50.2','192.168.50.1'),
    ('无人机','wlx502b73d008ac','50:2B:73:D0:08:AC','7669e0ae-ac55-4fc7-b1de-b6149352803a',
     'MicoAir_WiFi_Link_7191','192.168.4.2','192.168.4.1'),
)


def prepare_network(run,*,execute=subprocess.run):
    """One explicit preparation: inspect both bindings, reconnect only these profiles.

    Called in the existing child BEFORE any ROS/hardware owner opens. No ping,
    parameter write, profile edit, disconnect, global radio toggle or route edit.
    """
    run=Path(run);report={'phase':'checking','message':'正在检查两张专用 Tenda','adapters':[]}
    def save():
        tmp=run/'network_status.tmp'
        tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        tmp.replace(run/'network_status.json')
    def check_cancel():
        if (run/'network_cancel').exists():raise InterruptedError('已取消网卡准备；不会启动车机后端')
    def command(args,timeout=8):
        check_cancel()
        result=execute(args,capture_output=True,text=True,timeout=timeout)
        check_cancel()
        if result.returncode:
            raise RuntimeError((result.stderr or result.stdout or '命令失败').strip())
        return result.stdout
    def nm(fields,kind,identity):
        return command(['nmcli','--escape','no','-g',fields,kind,'show',identity]).splitlines()
    def device(spec):
        label,iface,mac,uuid,ssid,ip,peer=spec
        values=nm('GENERAL.TYPE,GENERAL.HWADDR,GENERAL.STATE,GENERAL.CON-UUID,IP4.ADDRESS','device',iface)
        if len(values)<4 or values[0]!='wifi' or values[1].upper()!=mac:
            raise RuntimeError('专用网卡身份不匹配，未切换其他网卡')
        return values
    def connected(values,spec):
        return values[2].startswith('100 ') and values[3]==spec[3] and any(
            v.strip()==spec[5]+'/24' for line in values[4:] for v in line.split(','))
    try:
        save()
        # Complete read-only validation of BOTH profiles before the first activation.
        for spec in DEDICATED_WIFI:
            label,iface,mac,uuid,ssid,ip,peer=spec
            report['message']=f'检查{label}专用网卡 {iface}';save()
            device(spec)
            values=nm('connection.interface-name,802-11-wireless.ssid,ipv4.never-default,ipv4.ignore-auto-dns,ipv4.gateway,ipv4.routes,ipv6.method','connection',uuid)
            if values!=[iface,ssid,'yes','yes','','','disabled']:
                raise RuntimeError('保存的热点绑定或路由/DNS隔离配置与基线不符，未修改配置')
        for spec in DEDICATED_WIFI:
            label,iface,mac,uuid,ssid,ip,peer=spec
            report['message']=f'检查{label}热点与 IPv4：{ssid}';save()
            values=device(spec);reconnected=False
            if not connected(values,spec):
                if values[3] not in ('','--',uuid):
                    raise RuntimeError('专用网卡正在使用其他连接，未强行切换；请先确认该网卡用途')
                report['message']=f'正在重连{label}热点 {ssid}（最多等待30秒，可停止准备）';save()
                command(['nmcli','--wait','30','connection','up','uuid',uuid,'ifname',iface],timeout=35)
                reconnected=True;values=device(spec)
            if not connected(values,spec):
                raise RuntimeError(f'未获得预期连接和 IPv4 {ip}/24；未启动车机后端')
            # Match the existing transports: car pins its interface, UAV binds
            # its source IPv4. A VPN's unrelated default route is not a failure.
            route_args=['ip','-j','-4','route','get',peer,'from',ip]
            if label=='小车':route_args+=['oif',iface]
            route=json.loads(command(route_args))
            # iproute2 emits "from" for an explicitly supplied source; a
            # preferred-source lookup instead emits "prefsrc" (or "src").
            if not route or route[0].get('dev')!=iface or route[0].get('prefsrc',route[0].get('src',route[0].get('from')))!=ip:
                raise RuntimeError(f'到 {peer} 的路由未使用本专用网卡；未修改其他网络')
            report['adapters'].append(dict(device=iface,role=label,ssid=ssid,ip=ip,reconnected=reconnected))
            save()
        check_cancel()
        report.update(phase='ready',message='两张专用网卡已就绪；继续读取车机实际反馈')
        save();return 0
    except Exception as exc:
        report.update(phase='cancelled' if isinstance(exc,InterruptedError) else 'failed',
                      message=f"{report['message']}：{exc}",raw=f'{type(exc).__name__}: {exc}')
        save();print(report['message'],flush=True);return 1

CONDITIONS={
    'battery_only_stationary_protected':'电池单独供电、USB 拔开；车机静止，位于防护隔离区。',
    'operator_present':'操作者在场；Pocket 开机、油门最低、SB 解锁关闭、SC 停机关闭。',
    'existing_holds_physically_reviewed':'已知桨叶仍安装；本轮现场条件已确认，不沿用历史拆桨假设。',
    'complete_parameter_backup_verified':'沿用当前配置绑定的完整参数备份；备份文件由原入口校验。',
    'no_motor_commands_approved':'本步骤只允许准备定位和遥测，不允许无人机解锁；小车另行点击就位。',
    'props_treated_as_installed':'人员离开桨叶及小车活动范围；不在上电时手动调整车机。',
    'current_rigid_body_and_extrinsics_reviewed':'Tracker 的 car01/MircoAir 实时有效，刚体、反光点与世界坐标未改，沿用当前外参。',
    'battery_only_wireless_verified':'确认无人机仅电池供电、USB 拔开，当前无线遥测正常。',
    'fresh_feedback_rates_verified':'本轮定位和回传已就绪；是否新鲜仍由原控制程序实时判定。',
    'approximate_layout_and_untuned_delay_accepted_for_initial_trial':'接受当前近似外参和未精调时延的试飞范围，不视为精确标定。',
    'onboard_loss_action_verified':'沿用此前已确认的飞控失联配置，当前未更改。',
    'protection_independent_of_pc_reviewed':'Pocket 接管/SC 停机及现场隔离措施可用，不依赖操作台窗口紧急停机。',
    'protected_flight_area_ready':'确认本轮起飞、车上方跟随、所选路线、旁侧返回和降落的全部净空。',
    'first_flight_envelope_reviewed':'已阅读本轮路线、速度、高度和边界；接受当前测试版本，不自动扩大场地。',
    'explicit_start_required':'此步骤只生成预览；最后再点击“确认起飞并运行”才允许启动电机。',
}


def review_conditions(flight=False,profile=None):
    from flight_execution_review import GROUND_REQUIRED,flight_confirmations
    keys=flight_confirmations(profile or json.loads(PROFILE.read_text())) if flight else GROUND_REQUIRED
    return {key:CONDITIONS.get(key,key) for key in keys}


def explain_error(raw,phase='准备连接'):
    """Known symptom -> bounded explanation. Never invent a hardware cause."""
    raw=str(raw)
    rules=[
        (('尚未确认安全收尾','缺少收尾确认'), '后端已退出，但尚未确认安全收尾；不能把退出码当作落地或停车证明。', '先确认车机实际状态，查看 flight_recovery、rates_recovery 和 joint_stop 记录，不自动重开。'),
        (('0x200','native_fault_bits": 512'), '小车控制器报告锁存故障，当前不能接受运动。', '保持车机停止，按提示重新给小车控制板上电；不要重启无人机。'),
        (('Address already in use','already owned','resource busy'), '通信端口或设备已被另一程序占用。', '关闭重复的联合入口或 QGC；本窗口不会强占端口。'),
        (('Cannot assign requested address','Network is unreachable','192.168.4.2'), '无人机专用网卡地址不可用或网络不可达。', '确认无人机 Tenda 已连接 WiFi-Link 热点；办公和小车网络不动。'),
        (('approach_geometry_required','approach_margin_below_arrival'), '绕行配置缺少有效外框尺寸或路径余量。', '核对小车、无人机外框及控制参考点偏移，补齐当前就位配置后重新准备。'),
        (('approach_start_overlaps','approach_vehicle_clearance_lost'), '小车已进入无人机的外框避让范围。', '本轮停止接近；停车后检查实体间距、定位和外参，重新安排有净空的起点。'),
        (('approach_final_spacing_too_small',), '设定的并排间距容不下两台设备外框及路径余量。', '增大并排间距；保留外框尺寸，不通过缩小模型绕过。'),
        (('approach_no_clear_path',), '当前区域边界内没有找到绕开无人机的接近路径。', '检查轨迹区域及现场净空，调整起点或明确可用区域；程序不会自动扩边。'),
        (('car_must_start_on_clear_left','clear_left_approach'), '当前使用的是旧版左侧直线就位配置。', '选择已填写外框尺寸的绕行配置，或按原左侧起点运行。'),
        (('outside_task_area','outside_flight_area','task_area','boundary'), '解析出的目标或位置超出了当前任务区域。', '查看轨迹和边界；先收尾，再调整起点或核对场地，不自动扩大区域。'),
        (('stationary_uav_moved','UAV_MOVED'), '就位期间无人机位置相对已记录起点发生变化。', '停止本轮，确认是否被移动或碰到；重新准备后再记录起点。'),
        (('subject_not_visible','occluded','Vicon connection','subject_stale','pose_stale','CAR_POSE_TIMEOUT'), '定位不可见或没有及时更新；仅凭此报文不能断定是相机、网络还是遮挡。', '查看 Tracker 刚体有效状态与定位数据年龄；不要移动设备来强行越过提示。'),
        (('TRANSPORT_SIGNED_AUTHORITY_TIMEOUT','CAR_TRANSPORT_LOST'), '小车控制链路未持续收到有效控制或反馈。', '等待原程序停车收尾，保留日志，检查小车供电和专用无线链路。'),
        (('Fresh disarmed Position','Ground Position','Position heartbeat'), '无人机尚未处于已锁定的 Position 模式。', '按屏幕提示用 SA 切换，保持油门最低、SB 关闭；无需重做摇杆测试。'),
        (('Confirmation text/age','Missing/used start preview','Start already consumed'), '起飞预览已过期、已使用或起点状态变化。', '本次没有自动重试；若尚未起飞，重新生成当前预览。'),
        (('Review expired','Review slot already','One explicit upgrade'), '当前会话的授权窗口已结束或已使用。', '不要重复起飞；停止并收尾后重新准备一轮。'),
        (('physical review','evidence missing','code_hash','hashes'), '当前配置、证据或代码与本轮确认记录不一致。', '保留原报文；先结束本轮，再用当前版本重新准备，不能复用旧授权。'),
        (('native_kill','native_stop','STOP_LATCHED'), '遥控停机开关触发或其状态尚不明确。', '保持油门最低，确认现场停止后按提示检查 SC；程序不会自动解除停机。'),
        (('manual_control_loss','input_rc','native_rc','handset'), '遥控输入尚未被确认，可能未连接或回传尚未更新。', '确认 Pocket 开机、接收机连接，保持开关不动，等待当前回传。'),
        (('LAND_TIMEOUT','landing_confirmation'), '尚未收到完整的落地和锁定确认。', '观察实际飞机并保留后端运行；必要时使用 Pocket 接管，不关闭通信程序。'),
        (('operator_cancel',), '已收到操作者的停止请求，正在走原有停车/降落收尾。', '等待落地锁定与小车停车确认；这不是立即空中停桨。'),
        (('preflight','Preflight Fail'), '飞控当前预飞检查尚未通过；具体项目以原始报文为准。', '查看下方报文及缺少项目，不反复点击起飞。'),
    ]
    for needles,reason,action in rules:
        if any(word in raw for word in needles):break
    else:
        reason='程序报告了尚未分类的异常；现有信息不足以确定根因。'
        action='保留本轮记录和原始报文，按当前停止/接管提示处理，不自动重试。'
    return dict(phase=phase,reason=reason,action=action,raw=raw)


def make_review(run,profile,plan,flight,consent):
    from flight_execution_review import draft,draft_ground
    required=review_conditions(flight,profile)
    if any(consent.get(key) is not True for key in required):
        raise PermissionError('尚未确认本轮现场条件')
    rec=json.loads((run/'session/flight_recovery.json').read_text())
    trial=profile['initial_trial'];transfer=trial['rigid_frame_transfer']
    factory=draft if flight else draft_ground
    record=factory(boot_session=rec['boot_session'],rigid_revision=trial['revision'],
        code_paths=[TOOLS/'rigid_body_revision.py',Path(__file__),TOOLS/'drone_console.py'],
        evidence_paths=[transfer['path'],transfer['baseline_profile']['path']],
        backup=Path(trial['evidence']['backup']['path']).parent,
        fixed_profile=profile,pilot_source='native_rc',air_ground_config=plan)
    now=time.time()
    record.update(authorized=True,issued_unix=now,expires_unix=now+175,
        confirmations={key:True for key in required},
        warning='Explicit GUI operator confirmation for this session only; backend current-state checks remain required.')
    record['confirmation_method']=consent.get('_confirmation_method','checklist')
    p=run/('execution_review.json' if flight else 'ground_review.json')
    with p.open('x') as f:json.dump(record,f,ensure_ascii=False,indent=2)
    return p


class JointConsoleRun:
    def __init__(self,*,popen=subprocess.Popen,review_factory=make_review,clock=time.monotonic):
        self.popen=popen;self.review_factory=review_factory;self.clock=clock
        self.proc=None;self.run=None;self.status={};self.status_at=None
        self.preview=None;self.preview_at=None;self.takeoff_sent=False
        self.ground_sent=False;self.execution_sent=False;self.cancelled=False
        self.issue=None;self.first_failure=None;self.stdout=None
        self.log=None;self.consent={};self.finished=False;self.cleanup_ok=False
        self.preview_requested=False;self.car_requested=False
        self.progress_log=None;self.progress_at=None;self.last_command=None
        self.network={}

    @property
    def running(self):return self.proc is not None and self.proc.poll() is None

    def launch(self,plan_path,consent,*,base=RUNS,existing=None):
        if self.running:raise RuntimeError('已有联合会话运行，不重复启动')
        if existing is not None:raise RuntimeError('已有其他联合会话；当前窗口只能观察，不重复占用设备')
        if self.proc is not None and not self.cleanup_ok:
            raise RuntimeError('上一会话未确认收尾，请保留记录并检查设备状态')
        profile=json.loads(PROFILE.read_text());plan=json.loads(Path(plan_path).read_text())
        required=review_conditions(False,profile)
        if any(consent.get(k) is not True for k in required):raise PermissionError('尚未确认本轮现场条件')
        from air_ground_resources import validate_bindings
        from ekf_static_profile import require_mission_profile
        validate_bindings(plan)
        require_mission_profile(profile,pilot_source='native_rc',height=plan['clearance_m'],hover_seconds=3.)
        run=Path(base)/('gui_joint_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));run.mkdir(parents=True)
        for name,value in (('plan.json',plan),('profile.json',profile),('operator_ground_consent.json',consent)):
            (run/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
        args=['/usr/bin/python3','-u','-B',str(TOOLS/'vicon_fixed_point_entry.py'),'--run','--handset','pocket',
            '--flight-console','--profile',str(run/'profile.json'),'--cooperative-plan',str(run/'plan.json'),
            '--site-ready','--output',str(run/'session')]
        network_args=['/usr/bin/python3','-u','-B',str(Path(__file__).resolve()),'--prepare-network',str(run)]
        command=shlex.join(network_args)+' && source /opt/ros/jazzy/setup.bash && source /home/czy/vicon_receiver_ws/drone_sim/isolated/vicon_b3_recovery_20261022_135653/install/local_setup.bash && exec '+shlex.join(args)
        output=(run/'backend_output.log').open('x')
        try:proc=self.popen(['bash','-c',command],cwd=TOOLS,stdin=subprocess.PIPE,stdout=output,
                           stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
        except BaseException:output.close();raise
        self.__init__(popen=self.popen,review_factory=self.review_factory,clock=self.clock)
        self.proc=proc;self.stdout=output;self.run=run;self.profile=profile;self.plan=plan;self.consent=dict(consent)
        # Lazy import avoids a second UI or resource owner, including in tests.
        from drone_console import LogTail
        self.log=LogTail(run/'session/console_events.jsonl')
        self.progress_log=LogTail(run/'session/cooperative.jsonl')
        return run/'session'

    def send(self,line):
        if not self.running or self.cancelled and line!='cancel':raise RuntimeError('当前会话不接受新的运行命令')
        if '\n' in line or '\r' in line:raise ValueError('one console command required')
        self.proc.stdin.write(line+'\n');self.proc.stdin.flush()
        self.last_command=line.split(' ',1)[0]
        with (self.run/'ui_actions.jsonl').open('a') as f:
            f.write(json.dumps(dict(at_ns=time.monotonic_ns(),command='TAKEOFF [one-use token]' if line.startswith('TAKEOFF ') else line),ensure_ascii=False)+'\n')

    def accept_event(self,row):
        text=row.get('text','')
        try:d=json.loads(text)
        except (ValueError,TypeError):d=None
        if isinstance(d,dict) and 'phase' in d:
            previous_stage=self.stage()
            self.status=d;self.status_at=float(row['at_ns'])/1e9
            fault=d.get('failure') or (d.get('cooperative') or {}).get('fault')
            if fault and fault!='operator_cancel':
                snapshot=d.get('failure_snapshot') or {}
                phase=snapshot.get('phase')
                # Preserve the user-facing stage; the unmodified core phase
                # remains available in the raw snapshot below.
                issue=explain_error(fault,previous_stage)
                issue['snapshot']=snapshot
                self.first_failure=self.first_failure or issue;self.issue=self.first_failure
        elif isinstance(d,dict) and 'confirmation_text' in d:
            if self.preview_requested and not self.takeoff_sent:
                self.preview=d;self.preview_at=self.clock();self.preview_requested=False
        elif isinstance(d,dict) and d.get('action')=='ground_alignment_requested':
            self.car_requested=True
        elif isinstance(text,str) and ('Command rejected:' in text or 'Service fault;' in text):
            self.preview_requested=False;self.issue=explain_error(text,self.stage())
            if self.last_command=='CAR_START':self.car_requested=False

    def poll(self):
        if self.proc is None:return
        network_path=self.run/'network_status.json'
        if network_path.exists():self.network=json.loads(network_path.read_text())
        if not self.running and not self.finished and self.network.get('phase') in ('failed','cancelled') and not (self.run/'session').exists():
            # No hardware owner was launched. This is not proof of vehicle state.
            self.finished=True;self.cleanup_ok=True
            if self.network['phase']=='failed':
                self.issue=dict(phase='专用网卡准备',reason=self.network['message'],
                    action='确认对应设备已上电、网卡已插入；处理后再点击准备连接。不会自动反复重试。',raw=self.network.get('raw'))
            if self.stdout:self.stdout.close()
            (self.run/'gui_result.json').write_text(json.dumps(dict(exit_code=self.proc.returncode,
                network_only=True,hardware_backend_started=False,network=self.network),ensure_ascii=False,indent=2)+'\n')
            return
        if self.running and not self.ground_sent and not self.cancelled:
            if (self.run/'session/flight_recovery.json').exists():
                # Only after the explicit launch consent; never retried silently.
                self.ground_sent=True
                try:self.send('ground '+str(self.review_factory(self.run,self.profile,self.plan,False,self.consent)))
                except (ValueError,KeyError,OSError,PermissionError,RuntimeError) as exc:
                    self.issue=explain_error(str(exc),'连接准备');self.stop()
        for _ in range(80):
            item=self.log.pop() if self.log else None
            if item is None:break
            self.accept_event(item[1])
        for _ in range(500):
            item=self.progress_log.pop() if self.progress_log else None
            if item is None:break
            self.progress_at=item[0]
        if not self.running and not self.finished:
            # Drain mirrored terminal events before deciding the outcome.
            if self.log and self.log.unread():return
            self.finished=True
            try:
                records=[json.loads((self.run/'session'/n).read_text()) for n in ('flight_recovery.json','rates_recovery.json','joint_stop.json')]
                self.cleanup_ok=all(d.get('verified_closed') and not d.get('needs_restore') for d in records[:2]) and records[2].get('car_stop_confirmed') is True and not records[2].get('errors')
            except (OSError,ValueError):self.cleanup_ok=False
            if self.proc.returncode!=0 or not self.cleanup_ok:
                raw=(self.run/'backend_output.log').read_text(errors='replace')[-6000:]
                if not self.cleanup_ok:raw='后端已退出，尚未确认安全收尾。\n'+raw
                self.issue=self.first_failure or explain_error(raw or '后端退出但缺少收尾确认',self.stage())
            if self.stdout:self.stdout.close()
            (self.run/'gui_result.json').write_text(json.dumps(dict(exit_code=self.proc.returncode,
                cleanup_verified=self.cleanup_ok,takeoff_sent=self.takeoff_sent,cancel_requested=self.cancelled,
                status=self.status,first_failure=self.first_failure,issue=self.issue),ensure_ascii=False,indent=2)+'\n')

    def current(self):
        # Status messages are emitted only on changes. The unchanged status
        # remains valid while THIS owner's continuous progress log is fresh.
        return self.status_at is not None and self.progress_at is not None and 0<=self.clock()-self.progress_at<=5.

    def can_align(self):
        c=self.status.get('cooperative') or {}
        return bool(self.running and not self.cancelled and not self.car_requested and self.current()
            and not self.status.get('missing') and not self.status.get('failure') and not self.status.get('operator_action')
            and c.get('car_task_phase')=='WAIT_START' and c.get('car_stopped'))

    def align(self):
        if not self.can_align():raise RuntimeError('尚未就绪，先按阶段指引处理；没有发送小车启动命令')
        self.issue=None;self.send('CAR_START');self.car_requested=True

    def can_preview(self):
        c=self.status.get('cooperative') or {}
        return bool(self.running and not self.cancelled and not self.takeoff_sent and self.current()
            and not self.status.get('missing') and not self.status.get('failure') and not self.status.get('operator_action')
            and self.status.get('phase')=='WAIT_EXPLICIT_START' and c.get('car_task_phase')=='WAIT_TAKEOFF' and c.get('car_stopped'))

    def request_preview(self,consent):
        if not self.can_preview():raise RuntimeError('尚未到起飞确认阶段，请先完成小车就位并选择 Position')
        if any(consent.get(k) is not True for k in review_conditions(True,self.profile)):
            raise PermissionError('尚未确认本轮飞行条件')
        self.issue=None;self.preview=None
        if not self.execution_sent:
            path=self.review_factory(self.run,self.profile,self.plan,True,consent)
            self.send('review '+str(path));self.execution_sent=True
        self.preview_requested=True;self.send('preview')

    def takeoff(self,preview,*,confirmed=False):
        if (confirmed is not True or not self.can_preview() or self.preview is None
                or preview!=self.preview or self.takeoff_sent or self.preview_at is None
                or self.clock()-self.preview_at>20):
            raise PermissionError('当前起飞预览未确认或已失效；没有发送起飞命令')
        self.takeoff_sent=True  # no retry after a potentially partial pipe write
        self.send(self.preview['confirmation_text']);self.preview=None

    def stop(self):
        if self.running and not self.cancelled:
            (self.run/'network_cancel').touch()
            self.send('cancel');self.cancelled=True;self.preview=None

    def stage(self):
        if self.proc is None:return '未启动'
        if self.finished and self.network.get('phase') in ('failed','cancelled') and not (self.run/'session').exists():return '网卡准备未完成'
        if self.finished:return '已收尾' if self.cleanup_ok else '收尾待确认'
        if not self.status and not (self.run/'session').exists():return '专用网卡准备'
        if self.cancelled:return '停车 / 降落收尾'
        d=self.status;c=d.get('cooperative') or {};phase=c.get('phase')
        if c.get('car_task_phase')=='TO_TAKEOFF':return '小车自动就位'
        if self.takeoff_sent:
            return {'TAKEOFF':'无人机起飞','RENDEZVOUS':'移至车上方','FOLLOW':'车机跟随',
                'WAIT_CAR_STOP':'小车停车','ALIGN':'返回起飞点','DESCEND':'缓慢下降','LAND':'接地与停桨',
                'COMPLETE':'任务完成 / 恢复回传'}.get(phase,'起飞交接 / '+str(d.get('phase','等待反馈')))
        if c.get('car_task_phase')=='WAIT_TAKEOFF':return '等待起飞确认'
        return '连接与定位准备'

    def guidance(self):
        if self.issue:return f"阶段：{self.issue['phase']}\n原因：{self.issue['reason']}\n处理：{self.issue['action']}"
        if self.network and not self.status and not (self.run/'session').exists():
            return '正在取消准备，等待当前连接命令退出；不会启动车机后端。' if self.cancelled and self.running else self.network['message']
        if self.finished:return '本轮已收尾，可查看记录或准备新一轮。' if self.cleanup_ok else '尚未确认安全收尾，请查看原始报文并观察车机。'
        if not self.running:return '①准备连接 → ②小车自动就位 → ③确认起飞 → 自动跟随、返回、缓降。'
        action=self.status.get('operator_action')
        if action:return action+' 保持 SB 关闭、油门最低。'
        if self.can_align():return '连接和静止准备已就绪。确认接近路径无人无遮挡后，点击“②小车自动就位”；无人机不会起飞。'
        if self.can_preview():return '小车已到位停车。点击“③生成起飞确认”，阅读本轮预览后再确认启动电机。'
        missing=self.status.get('missing') or []
        if missing:
            groups=[]
            for key,label in [('heartbeat','飞控无线回传'),('local_position','无人机位置'),('input_rc','遥控输入'),
                ('native_rc','遥控输入'),('preflight','预飞结果'),('fusion','定位融合'),('ekf','估计器状态'),
                ('landed','落地状态'),('actuator','电机锁定状态'),('commander','飞控状态')]:
                if any(key in m for m in missing) and label not in groups:groups.append(label)
            return '正在等待：'+('、'.join(groups) if groups else '当前设备状态')+'。原始缺少项目可在报文区查看。'
        if self.takeoff_sent:return '本轮正在由原联合程序运行。需要提前结束时点“停止并收尾”；紧急情况使用 Pocket 接管/SC。'
        return '等待原联合程序回传当前状态；不会自动跳过阶段或起飞。'


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description='操作台内部专用网卡准备；不启动运动')
    parser.add_argument('--prepare-network',required=True,type=Path)
    raise SystemExit(prepare_network(parser.parse_args().prepare_network))
