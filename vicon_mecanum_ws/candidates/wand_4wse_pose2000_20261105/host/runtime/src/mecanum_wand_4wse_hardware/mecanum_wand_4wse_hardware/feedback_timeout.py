from mecanum_wand_4wse_control.feedback_policy import (
    DIAGNOSTIC_TIMEOUT_S,WHEEL_TIMEOUT_S,COMBINED_FEEDBACK_TIMEOUT_S)
from mecanum_vicon_control.transport_protocol import MessageType

FEEDBACK_TIMEOUT_S=COMBINED_FEEDBACK_TIMEOUT_S

def sample_timeout_s(kind):
    if kind==MessageType.LINK_STATUS:return DIAGNOSTIC_TIMEOUT_S
    if kind==MessageType.WHEEL_TELEMETRY:return WHEEL_TIMEOUT_S
    raise ValueError('only native diagnostic and wheel samples have feedback budgets')
