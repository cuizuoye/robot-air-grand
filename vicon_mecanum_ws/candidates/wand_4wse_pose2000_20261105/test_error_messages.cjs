'use strict';
// Pure presentation tests, including the three observed failures. No IO/control.
const assert=require('node:assert/strict');
const zh=require('./host/web/error_messages.js');
const cases=[
 ['CAR_POSE_TIMEOUT','小车定位更新超时'],
 ['WAND_POSE_TIMEOUT','标定杆定位更新超时'],
 ['PREPARE_BLOCKED: 状态已撤销或存在故障','准备流程已中止'],
 ['SIGN_GEOMETRY_OR_ATTITUDE_FLIP','定位姿态或点位几何异常'],
 ['NATIVE_FAULT_0x200','软件急停已锁存'],
 ['NATIVE_FAULT_0x100','STM32主控报告故障'],
 ['WHEEL_SPEED_FEEDBACK_TIMEOUT','轮速反馈更新超时'],
 ['DIAGNOSTIC_STALE','主控诊断反馈更新超时'],
 ['FEEDBACK_TIMEOUT','小车反馈更新超时'],
 ['SERVER_CONTROL_LOOP_GAP','电脑控制调度中断'],
 ['CONTROL_TIMER_GAP','电脑控制调度中断'],
 ['TRANSPORT_SIGNED_AUTHORITY_TIMEOUT','末级已验签运动指令更新超时'],
 ['OPERATOR_LEASE_TIMEOUT','有效控制指令未及时更新'],
 ['VIEW_DISCONNECTED','操作台通信连接中断'],
 ['DENIED_OWNER_EXISTS_OR_ENDED','已有操作会话或本轮已结束'],
 ['START_EXPIRED_OR_REPLAYED','启动请求已失效或尚未就绪'],
 ['SOURCE_SAME_ORDER_CONFLICT','定位或控制数据来源冲突'],
 ['SIGN_HORIZONTAL_DEGENERACY','标定杆水平朝向无法确定'],
 ['STOP_UNCONFIRMED','尚未收到完整停车确认'],
 ['TIMER_COMPLETE','演示计时结束'],
 ['STAGE6_LONG600_TIME_COMPLETE','演示计时结束'],
 ['CLIENT_SECOND_SPACE','已按空格结束本轮'],
 ['CLIENT_ESC','操作员已停止'],
 ['RuntimeError: secret_function_name()','未分类的软件异常'],
 ['NEW_UNKNOWN_TIMEOUT_CODE','未分类的软件异常'],
];
let tests=0;
for(const [code,title] of cases){
 const result=zh.describe(code);assert.equal(result.title,title,code);
 assert(result.detail&&result.action,code);
 assert(!/[A-Za-z]+Error|[A-Z]+_[A-Z_]+|secret_function_name/.test(Object.values(result).join(' ')),code);
 tests++;
}
assert.equal(zh.primary({channel:{fault:'PREPARE_BLOCKED: 状态已撤销'},reason:'WAND_POSE_TIMEOUT'}),'WAND_POSE_TIMEOUT');tests++;
assert.equal(zh.primary({fault:'OPERATOR_ESTOP',reason:'WAND_POSE_TIMEOUT'}),'OPERATOR_ESTOP');tests++;
assert.equal(zh.state('ACTIVE'),'跟随中');tests++;
assert.equal(zh.instruction('已结束：WAND_POSE_TIMEOUT；请检查'),'已结束：标定杆定位更新超时；请检查');tests++;
assert(!zh.instruction('软件异常：RuntimeError: secret_function_name()').includes('RuntimeError'));tests++;
assert(zh.facts({pose_source_ages_s:{car:.23455,wand:.200504},native_fault_bits:0}).includes('200.5毫秒'));tests++;
assert(zh.facts({pose_source_ages_s:{}}).includes('暂无'));tests++;
assert(!zh.facts({pose_source_ages_s:{car:NaN,wand:Infinity}}).includes('NaN'));tests++;
assert.equal(zh.describe('explicit start required').title,'等待人工启动');tests++;
console.log(JSON.stringify({passed:true,tests,hardware_socket_opened:false,ros_initialized:false}));
