'use strict';
// Presentation only: raw protocol codes remain unchanged in audit logs.
(function(root){
  const rules=[];
  const add=(pattern,title,detail,action)=>rules.push({pattern,title,detail,action});
  const tracking='确认Tracker中的模型随动作实时更新、标记点无遮挡；若仍出现，检查Vicon接收及ROS处理。无需反复重启小车。';
  const software='保持小车停止，关闭旧页面后重新检查连接；若复现，保留本页和日志，不要反复启动。';
  add(/^explicit start required$/,'等待人工启动','检查和连接不会自动使能。','准备就绪后轻按并松开空格开始。');
  add(/^waiting for a new held-enable edge$/,'等待新的人工启动动作','已完成准备，尚未开始运动。','轻按并松开空格开始。');
  add(/^(?:TIMER_COMPLETE|STAGE\d+_.*TIME_COMPLETE)$/,'演示计时结束','本轮已达到设定时长。','确认停稳后，可重新打开页面进行下一轮。');
  add(/(?:^ESTOP$|OPERATOR_ESTOP|NATIVE_FAULT_0x200|FAULT_ESTOP)/,'软件急停已锁存','主控收到了急停指令；普通重新准备不会清除锁存。','确认实车完全停稳后，按独立故障恢复流程处理；不要反复启动。');
  add(/(?:SECOND_SPACE)/,'已按空格结束本轮','系统收到了第二次空格停止指令。','确认停稳后，可重新打开页面进行下一轮。');
  add(/(?:^ESC$|CLIENT_ESC|STOP_BUTTON|LOCAL_OPERATOR_STOP|LOCAL_STOP|operator stop|OPERATOR_STOP|CLIENT_STOP)/i,'操作员已停止','系统收到了停止按钮、Esc或终端停止指令。','确认停稳后，可重新打开页面进行下一轮。');
  add(/PAGE_CLOSED/,'操作页面已关闭','本轮因页面关闭而结束。',software);
  add(/CAR_POSE_TIMEOUT|SELECTED_CAR_STALE|SOURCE_CAR_AGE/,'小车定位更新超时','最近有效的小车定位数据超过2000毫秒；不等同于小车Wi-Fi断开，也不能仅据此认定被遮挡。',tracking);
  add(/WAND_POSE_TIMEOUT|SIGN_PACKET_AGE_OR_MIXED_FRAME/,'标定杆定位更新超时','未及时得到有效的Sign定位及五点同帧数据；定位保护时限为2000毫秒。',tracking);
  add(/SIGN_GEOMETRY|ATTITUDE_FLIP|CAR_ATTITUDE|POSE_FLIP|TILT/,'定位姿态或点位几何异常','收到的姿态、反光点间距或倾斜量不符合当前标定；这并不证明实物真的翻转。','确认Tracker中刚体及点名正确、点位固定；重新建立过刚体时需核对标定。');
  add(/HORIZONTAL_DEGENERACY/,'标定杆水平朝向无法确定','杆的参考方向过于接近竖直，无法可靠计算水平旋转角。','将杆恢复到可辨认水平朝向的姿态，保持五点可见，再重新检查。');
  add(/WAND_ANGLE_AMBIGUOUS/,'标定杆转角方向不确定','相邻有效数据接近半圈跳变，无法可靠确定旋转方向。','保持杆可见，避免瞬间半圈翻转；检查点位识别是否稳定。');
  add(/(?:CAR|WAND)_POSE_JUMP|POSITION_JUMP/,'定位发生跳变','相邻有效定位变化超过允许范围；可能来自刚体误识别或数据空窗。',tracking);
  add(/LIFT|LIFTED/,'检测到车体高度异常','定位显示车体高度超出地面范围，可能被抬起或刚体姿态异常。','保持车体落地并确认Tracker中高度、姿态正确，检查标记点固定情况。');
  add(/NATIVE_FAULT_/,'STM32主控报告故障','主控反馈含故障位；具体位值保留在本轮日志中。','先确认小车停稳，再根据故障位处理；不要通过反复准备绕过故障。');
  add(/WHEEL.*(?:TIMEOUT|STALE|GAP)|(?:TIMEOUT|STALE).*WHEEL/,'轮速反馈更新超时','未在500毫秒内收到有效的新轮速反馈；不代表Wi-Fi名称消失。','保持停车，检查ESP32转发、Tenda反馈回程和ROS收包处理。');
  add(/DIAG.*(?:TIMEOUT|STALE|GAP)|(?:TIMEOUT|STALE).*DIAG/,'主控诊断反馈更新超时','未在500毫秒内收到有效的新诊断报文。','保持停车，检查ESP32转发、Tenda反馈回程和ROS处理；无需反复重启Vicon。');
  add(/FEEDBACK.*(?:TIMEOUT|STALE|GAP)|(?:TIMEOUT|STALE).*FEEDBACK/,'小车反馈更新超时','有效反馈报文未在规定时间内更新；当前反馈保护为500毫秒。','保持停车，检查无线回程、ESP32转发和ROS接收处理。');
  add(/STOP_UNCONFIRMED|STOP_CONFIRM.*TIMEOUT/,'尚未收到完整停车确认','主机已结束跟随，但现有数据不足以确认实车已经停稳。','立即查看实车；若仍运动请断开动力电源。确认停稳前不要重启演示。');
  add(/CONTROL_TIMER_GAP|SERVER_CONTROL_LOOP_GAP|BACKEND_COMMAND_CHANNEL_TIMEOUT/,'电脑控制调度中断','后台控制循环或本机指令传递超过100毫秒未及时处理。','检查电脑负载、阻塞进程和系统日志；关闭不必要的高负载程序后重新检查。');
  add(/TRANSPORT_SIGNED_AUTHORITY_TIMEOUT/,'末级已验签运动指令更新超时','硬件通信模块没有及时收到新的、通过来源校验的运动指令；这与2000毫秒定位超时不是同一个检查。','保持停车，检查ROS末级输出发布、验签接收和调度；不要仅靠重启小车处理。');
  add(/COMMAND_TIMEOUT|OPERATOR_LEASE_TIMEOUT|LEASE.*(?:TIMEOUT|EXPIRED)/,'有效控制指令未及时更新','控制链未及时得到有效新指令或授权；不是演示时长耗尽。',software);
  add(/FINAL_GATE|GATE_STATUS|GATE_.*FAULT/,'末级输出校验未通过','独立输出检查没有确认当前指令可安全发送。',software);
  add(/COMMAND_REPLAY|START_EXPIRED_OR_REPLAYED|START_NOT_READY/,'启动请求已失效或尚未就绪','此启动请求没有被接受；重复或迟到请求不会补启动。','等待页面显示准备就绪，再轻按并松开空格；已结束的页面需重新打开。');
  add(/OWNER_EXISTS|OTHER_WINDOW|ENDED_SESSION/,'已有操作会话或本轮已结束','当前页面不能接管其他窗口，也不能重新使用已结束会话。','关闭旧跟随窗口，确认停稳后从启动指令打开新页面。');
  add(/VIEW_PAIR_TIMEOUT|VIEW_ACK_TIMEOUT/,'显示通道连接或回应超时','浏览器显示通道没有及时完成配对或回应。',software);
  add(/DISCONNECT|CONNECTION_ERROR|TRANSPORT_EOF/,'操作台通信连接中断','浏览器与本机后台之间的通道断开；不等同于小车无线断开。',software);
  add(/BACKPRESSURE|QUEUE|UNBOUNDED/,'软件通信队列积压','软件处理未跟上数据到达速度，当前会话不能继续。','保持停止，检查电脑资源及后台队列；保留日志以定位阻塞。');
  add(/SOURCE_.*CONFLICT|PUBLISHER|PROVENANCE|GRAPH|GID/,'定位或控制数据来源冲突','数据来源与本轮期望身份不一致，或同一ROS域存在重复来源。','检查域0内的接收器和旧控制入口；其他ROS域的无人机接收器不应被停止。');
  add(/CLOCK|STAMP|FUTURE|REPLAY|MIXED_FRAME|SELECTED_.*STALE/,'数据时间戳或帧匹配异常','收到的数据可能过期、重复、乱序或不属于同一帧。','保持停车，检查Vicon接收与ROS帧匹配；不要用旧数据刷新有效期。');
  add(/SIGN_|SOURCE_BUNDLE|SOURCE_CAR|DIGEST|SCHEMA|SEQUENCE|AUTHORITY_MODE|ACTION_NOT_ALLOWED|NON_KEY_INPUT|DATA_INVALID|DENIED_/,'数据格式或来源校验失败','软件收到的数据与当前控制协议不一致；详细原始信息已保留在日志。',software);
  add(/ACTUATOR|MOTOR|STALL|OVERSPEED/,'执行器反馈异常','主控或轮速反馈触发了运动异常判定；需要结合原始故障位确认原因。','先确认停稳，再检查对应轮组及反馈；不要直接重复启动。');
  add(/PREPARE_BLOCKED/,'准备流程已中止','准备条件发生变化或系统已进入故障状态；本轮未自动恢复。','查看下方最初原因，恢复定位或通信后关闭旧页面，重新检查。');
  add(/PREPARE|ARM_TIMEOUT|READY_TIMEOUT/,'未能完成准备','定位、通信、时钟或零输出确认尚未全部满足。','保持Sign稳定可见，查看具体阻断原因；本轮不会自动运动。');
  add(/WORKER|SERVER_TERMINATED|LAUNCHER_EXITED|WAITING_FOR_BROWSER_TIMEOUT/,'演示后台已退出或等待超时','后台进程终止，或页面未在等待时限内连接。',software);
  add(/BOUNDARY|GEOFENCE|FENCE|RADIUS/,'运动范围检查未通过','当前入口报告了范围限制；请核对是否使用了旧版本入口。','保持停止并核对启动命令；不要在未确认入口前重复启动。');
  add(/FRAME|POSE_DECODE|NONFINITE|QUATERNION/,'定位数据格式异常','坐标系、位置或姿态数值不符合当前配置。',tracking);
  add(/WINDOW_BLUR|WINDOW_HIDDEN|WINDOW_NOT_FOREGROUND/,'旧入口触发窗口焦点保护','当前后台独立会话不应因切换窗口停止；请核对是否使用了旧入口。',software);
  const states={WAITING:'等待数据',STANDBY:'待机',ARMED:'已准备',READY:'准备就绪',ACTIVE:'跟随中',
    STOPPING:'正在停车',FAULT:'故障停机',PAUSED:'已暂停',STOPPED_CONFIRMED:'已确认停稳',
    STOP_UNCONFIRMED:'停车尚未确认',READONLY:'只读观察',COMPLETED:'已完成',DISABLED:'未使能'};
  function id(s){let n=2166136261;for(const c of s)n=Math.imul(n^c.charCodeAt(0),16777619);return String(n>>>0);}
  function describe(reason){
    const raw=String(reason||'').slice(0,2000);
    if(!raw)return {title:'无',detail:'',action:'',known:true};
    const found=rules.find(rule=>rule.pattern.test(raw));
    if(found)return {title:found.title,detail:found.detail,action:found.action,known:true};
    return {title:'未分类的软件异常',detail:'详细信息已保存在本轮日志，诊断编号'+id(raw)+'。',action:software,known:false};
  }
  function primary(value){
    const fault=String(value?.channel?.fault||value?.fault||'');
    const cause=String(value?.reason||'');
    // A preparation wrapper must not hide the supervisor's first stop cause.
    return fault.startsWith('PREPARE_BLOCKED')&&cause&&!cause.startsWith('PREPARE_BLOCKED')
      ?cause:(fault||cause);
  }
  function instruction(value){
    let s=String(value||'');
    if(!s)return '';
    if(!/[\u3400-\u9fff]/.test(s))return describe(s).title;
    s=s.replace(/armed; hold to capture current origins; no output yet/g,'已准备；新的启动动作后才记录原点，当前无运动输出')
      .replace(/explicit start required/g,'等待人工启动')
      .replace(/waiting for a new held-enable edge/g,'等待新的人工启动动作');
    if(/(?:[A-Za-z]+Error|Traceback|\bfunction\b)/.test(s))return describe(s).title;
    return s.replace(/\b[A-Z][A-Z0-9]*(?:_[A-Za-z0-9]+)+(?::[^；\n]*)?/g,code=>describe(code).title);
  }
  function facts(d){
    if(!d)return '';
    const out=[];
    for(const [key,label] of [['car','小车'],['wand','标定杆']]){
      const age=d.pose_source_ages_s?.[key];
      if(typeof age==='number'&&Number.isFinite(age)&&age>=0)out.push(`${label}最近有效定位：${(age*1000).toFixed(1)}毫秒前`);
      else out.push(`${label}：暂无有效定位年龄记录`);
    }
    if(Number.isSafeInteger(d.native_fault_bits))out.push(`主控故障位：${d.native_fault_bits}`);
    return out.join('；');
  }
  const api={describe,primary,instruction,facts,state:value=>states[value]||'状态待确认'};
  root.ErrorChinese=api;
  if(typeof module!=='undefined')module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);
