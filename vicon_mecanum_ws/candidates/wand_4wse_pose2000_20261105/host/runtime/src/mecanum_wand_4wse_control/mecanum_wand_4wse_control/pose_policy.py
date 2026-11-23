"""Explicit user-selected Vicon source-age allowance, not a Vicon mandate.

This is ONLY localization freshness, never command/lease/feedback timing.
Both original source time and first receipt time must remain within the bound.
No caller may refresh either time to make a duplicate frame look new.
"""
POSE_TIMEOUT_S = 2.0
POSE_TIMEOUT_MS = 2000

