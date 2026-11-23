"""旧版PX4地面诊断：保持Shell，不以销毁会话作为命令分隔符。"""
import re

_ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
_PROMPT = re.compile(r'(?:^|\n)nsh>[ \t]*\Z')


def reply_buffer_limit(payload, *, ground_check=False):
    # Explicit 48-publication ground diagnostic only; all replies stay bounded.
    return 65536 if ground_check and payload == b'listener vehicle_visual_odometry -n 48\n' else 24000


def current_reply(text, payload):
    """必须看见本条命令回显之后的终端提示符，不能只凭部分字段返回。

    初始提示符、旧命令输出、截断回复均不接受；超时由调用方停发/回退，
    不发送flags=0或Ctrl-C来强制终止Shell。
    """
    if not isinstance(payload, bytes) or not payload.endswith(b'\n'):
        return None
    clean = _ANSI.sub('', text).replace('\r', '')
    command = payload.decode('ascii').rstrip('\n')
    echo = re.search(r'(?:^|\n)[ \t]*(?:nsh>[ \t]*)?' + re.escape(command) + r'\n', clean)
    return clean[echo.start():] if echo and _PROMPT.search(clean[echo.end():]) else None


def reply_complete(text, payload):
    return current_reply(text, payload) is not None
