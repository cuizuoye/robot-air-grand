"""Source-age budgets, not socket processing deadlines or refreshed stamps.

Diagnostic and wheel reports may be500ms old. The raw receive gap uses the
same budget, and a combined ROS report retains the OLDEST actual source time.
Standstill proof spacing is independently preserved at300ms; it is NOT another
running watchdog. No command, pose, firmware lease or control-loop change.
"""
DIAGNOSTIC_TIMEOUT_S=.500
WHEEL_TIMEOUT_S=.500
COMBINED_FEEDBACK_TIMEOUT_S=max(DIAGNOSTIC_TIMEOUT_S,WHEEL_TIMEOUT_S)
STOP_COVERAGE_MAX_GAP_S=.300
