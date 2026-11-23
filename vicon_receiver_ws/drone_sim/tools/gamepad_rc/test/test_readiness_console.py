import io
from pathlib import Path
import sys
import threading
import time

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from preflight_session_status import CHECKS, describe
from readiness_console import LatestStatusPrinter, StableReadiness, render


def status(complete=True):
    checks={name:complete for name,_,_ in CHECKS}
    return {'elapsed_s':1.,'session':describe(checks),
            'identity_fresh_disarmed':complete,'home_input_monitor_running':complete}


def test_all_conditions_need_one_continuous_second():
    gate=StableReadiness();s=status()
    assert not gate.observe(s,0)
    assert not gate.observe(s,.99)
    assert gate.observe(s,1.)


def test_missing_or_degraded_observation_resets_completion_timer():
    gate=StableReadiness();s=status()
    assert not gate.observe(s,0)
    assert not gate.observe(status(False),.8)
    assert not gate.observe(s,1.)
    assert not gate.observe(s,1.9)
    s['session']=describe({name:True for name,_,_ in CHECKS},fault='latched')
    assert not gate.observe(s,2.1)


@pytest.mark.parametrize('t',[-1,float('nan'),float('inf')])
def test_invalid_time_cannot_finish(t):
    with pytest.raises(ValueError):StableReadiness().observe(status(),t)


def test_backwards_time_cannot_finish():
    gate=StableReadiness();gate.observe(status(),2)
    with pytest.raises(ValueError):gate.observe(status(),1)


def test_console_lists_all_missing_conditions_and_never_calls_it_flight_ready():
    text=render(status(False))
    assert text.count('缺少：')==len(CHECKS)
    assert '不是起飞许可' in text and 'HOME监听已就绪' not in text
    s=status();s['session']['checks']['home_release_lock_verified']=False
    assert '请短按一次HOME并松开' in render(s)


def test_slow_output_cannot_block_serial_owner_publishing_updates():
    entered=threading.Event();release=threading.Event()
    class Slow:
        def write(self,_):entered.set();release.wait(2)
        def flush(self):pass
    printer=LatestStatusPrinter(Slow())
    try:
        printer.publish('first');assert entered.wait(1)
        started=time.monotonic()
        for i in range(1000):printer.publish(str(i))
        assert time.monotonic()-started<.5
        assert printer.queue.qsize()<=1
        printer.close()
    finally:release.set();printer.worker.join(1)


def test_closed_printer_cannot_enqueue_new_updates():
    printer=LatestStatusPrinter(io.StringIO());printer.close()
    printer.publish('late')
    assert not printer.worker.is_alive()
