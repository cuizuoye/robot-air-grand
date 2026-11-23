"""Offline scheduler regression; no serial/ROS or flight authorization."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from compact_vision import next_parameter_read, parameter_freshness_details


def simulate(drop, duration=10):
    names=[f'p{i}' for i in range(31)]
    received={n:0. for n in names};requested={};pending=[];counts={};peak=0
    for tick in range(int(duration*1000)):
        now=tick/1000
        for n,at in pending[:]:
            if at<=now:
                received[n]=now;pending.remove((n,at))
        if tick%67==0:
            n=next_parameter_read(names,received,requested,now)
            if n is not None:
                if n in requested:assert now-requested[n]>=.4
                requested[n]=now;counts[n]=counts.get(n,0)+1
                if not drop(n,counts[n]):pending.append((n,now+.12))
        peak=max(peak,max(now-v for v in received.values()))
    return peak,received,requested,counts


def test_one_lost_reply_retried_before_four_second_limit():
    peak,_,_,counts=simulate(lambda n,c:n=='p0' and c==1)
    assert peak<4
    assert all(v>=2 for v in counts.values()) and len(counts)==31


def test_several_lost_replies_do_not_starve_other_parameters():
    peak,_,_,counts=simulate(lambda n,c:n in ('p0','p7','p15') and c<=2)
    assert peak<4 and len(counts)==31


def test_permanently_missing_reply_stays_stale_not_refreshed_by_requests():
    peak,received,requested,_=simulate(lambda n,c:n=='p0',duration=5)
    report=parameter_freshness_details(['p0'],received,requested,5)['p0']
    assert peak>=4 and report['fresh'] is False and report['reply_age_s']==5
    assert report['request_age_s']<1


def test_missing_and_future_samples_fail_without_nan_json():
    import json
    report=parameter_freshness_details(['missing','future','boundary'],
        {'future':11,'boundary':6},{},10)
    assert all(not row['fresh'] for row in report.values())
    assert report['missing']['reply_age_s'] is None
    json.dumps(report,allow_nan=False)


def test_retry_cooldown_and_oldest_priority():
    assert next_parameter_read(['a','b'],{'a':1,'b':2},{'a':2.9},3)=='b'
    assert next_parameter_read(['a'],{}, {'a':2.9},3) is None
    assert next_parameter_read(['a','b'],{'a':1,'b':2},{},3)=='a'


def test_recent_reply_is_not_polled_just_to_fill_rate_budget():
    assert next_parameter_read(['a'],{'a':2.8},{},3,minimum_reply_age=.6) is None
    assert next_parameter_read(['a'],{'a':2.0},{},3,minimum_reply_age=.6)=='a'
