"""File-only preflight used by direct Node hardware invocation, before listen."""
import json
import os
import bootstrap
if __name__=='__main__':
    if os.environ.get('WAND_4WSE_MODE')!='hardware':raise ValueError('hardware admission mode required')
    rt=bootstrap.runtime()
    if rt.software_only or rt.config.stage!=6:raise ValueError('exact STOPFIX stage6 hardware profile required')
    if float(os.environ.get('WAND_4WSE_MAX_ACTIVE_S','nan'))!=rt.profile['commissioning_review']['max_active_s']:
        raise ValueError('requested duration differs from reviewed profile')
    print(json.dumps(dict(passed=True,hardware_opened=False,mode=rt.mode,namespace=rt.namespace,physical_qualified=False)))
