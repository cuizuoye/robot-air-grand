from one_home_preflight_policy import StartupHome
from static_lock_gate import StaticLockGate
from test_lock_clock_probe import ready, sample, reply


def prepared():
    home=StartupHome();home.event(1,.8);home.event(0,.9);home.ack(.95,boot_us=1)
    lock=StaticLockGate();lock.clock=ready();lock.status.sample=sample()
    return home,lock


def test_short_proof_requires_current_post_home_source_and_clock():
    home,lock=prepared()
    assert home.observe_short(lock,now_ns=1_050_000_000,post_home_ns=950_000_000,identity_verified=True)
    assert home.short_barrier_ns==5_000_000_000


def test_ack_alone_or_freshly_received_pre_home_source_not_proof():
    home,lock=prepared();lock.status.sample=sample(4990)
    assert home.short_bootstrap_ready() and not home.verified
    assert not home.observe_short(lock,now_ns=1_050_000_000,post_home_ns=950_000_000,identity_verified=True)


def test_clock_request_sent_before_release_is_not_barrier():
    home,lock=prepared()
    assert not home.observe_short(lock,now_ns=1_050_000_000,post_home_ns=1_010_000_000,identity_verified=True)
    assert home.short_barrier_ns is None


def test_identity_or_aborted_home_never_pass():
    for abort in (False,True):
        home,lock=prepared()
        if abort:home.event(1,1.)
        assert not home.observe_short(lock,now_ns=1_050_000_000,post_home_ns=950_000_000,identity_verified=abort)


def test_missing_release_never_allows_telemetry_bootstrap():
    home=StartupHome();home.event(1,.8);home.ack(.95)
    assert not home.short_bootstrap_ready()
