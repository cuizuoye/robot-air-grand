"""Bounded console updates and one-second stable, NON-FLIGHT completion."""
import math
import queue
import sys
import threading


def render(status):
    session=status['session']
    lines=[f"[{status['elapsed_s']:.1f}s] 台架连接检查：{session['phase']}（不是起飞许可）"]
    if session['latched_fault']:
        lines.append('本次故障已锁存：'+str(session['latched_fault'])+'；不会自动判定成功，需退出后检查。')
    missing=session['missing_observations']
    lines.extend('  缺少：'+item['hint'] for item in missing)
    if not missing and not session['latched_fault']:
        lines.append('  各项观测已齐全，等待连续稳定1秒……')
    if (status.get('identity_fresh_disarmed') is True
            and session['checks']['actuator_baseline_seen']
            and status.get('home_input_monitor_running') is True
            and not session['checks']['home_release_lock_verified']):
        lines.append('  HOME监听已就绪：请短按一次HOME并松开；仅验证未解锁输出锁定，不启动电机。')
    return '\n'.join(lines)


class StableReadiness:
    def __init__(self):
        self.since=self.last=None

    def observe(self,status,now):
        if not math.isfinite(now) or now<0 or (self.last is not None and now<self.last):
            raise ValueError('Finite monotonic readiness clock required')
        self.last=now
        s=status['session']
        eligible=(s.get('phase')=='STATIC_BENCH_INPUTS_OBSERVED'
                  and s.get('static_bench_inputs_observed') is True
                  and s.get('latched_fault') is None)
        if not eligible:
            self.since=None
            return False
        if self.since is None:self.since=now
        return now-self.since>=1.


class LatestStatusPrinter:
    """One pending update; a slow terminal must not block the serial owner."""
    def __init__(self,stream=None,*,capacity=1,close_stream=False):
        self.stream=stream if stream is not None else sys.stdout
        self.queue=queue.Queue(maxsize=capacity)
        self.close_stream=close_stream
        self.closed=False
        self.worker=threading.Thread(target=self._write,daemon=True)
        self.worker.start()

    def _write(self):
        try:
            while True:
                item=self.queue.get()
                if item is None:return
                try:
                    self.stream.write(item+'\n');self.stream.flush()
                except (OSError,ValueError):return
        finally:
            if self.close_stream:self.stream.close()

    def _replace(self,item):
        try:self.queue.get_nowait()
        except queue.Empty:pass
        try:self.queue.put_nowait(item)
        except queue.Full:pass

    def publish(self,text):
        if not self.closed:
            try:self.queue.put_nowait(text)
            except queue.Full:self._replace(text)

    def close(self,*,drain=False):
        self.closed=True
        if drain:
            try:self.queue.put(None,timeout=.1)
            except queue.Full:self._replace(None)
        else:self._replace(None)
        self.worker.join(timeout=.1)
