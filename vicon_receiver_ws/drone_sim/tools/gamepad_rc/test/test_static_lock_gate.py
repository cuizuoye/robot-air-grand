from test_lock_clock_probe import ready, sample, reply
from static_lock_gate import StaticLockGate


def gate(stamp=5005):
    value = StaticLockGate()
    value.clock = ready()
    value.status.sample = sample(stamp)
    return value


def test_current_permission_expires_and_recovery_never_auto_resumes():
    value = gate()
    assert value.begin(1_050_000_000, identity_verified=True)
    assert not value.assess(1_550_000_000, identity_verified=True)['valid']
    value.clock = ready()
    value.status.sample = sample()
    assert not value.assess(1_050_000_000, identity_verified=True)['valid']
    assert not value.begin(1_050_000_000, identity_verified=True)


def test_uncertainty_requires_new_reply_not_waiting_alone():
    value = gate(5050)
    assert not value.begin(1_050_000_000, identity_verified=True)
    assert not value.assess(1_100_000_000, identity_verified=True)['valid']
    value.clock.request(1_100_000_000)
    assert value.ingest(reply(remote=5_100_000_000, token=1_100_000_000), 1_120_000_000)
    assert value.begin(1_120_000_000, identity_verified=True)


def test_unsafe_latest_state_never_falls_back_to_previous_good_state():
    value = gate()
    assert value.begin(1_050_000_000, identity_verified=True)
    value.status.sample['bits'] |= 2
    assert not value.assess(1_060_000_000, identity_verified=True)['valid']
    value.status.sample['bits'] &= ~2
    assert not value.assess(1_070_000_000, identity_verified=True)['valid']


def test_identity_loss_and_explicit_stop_latch():
    for reason in ('identity', 'stop'):
        value = gate()
        assert value.begin(1_050_000_000, identity_verified=True)
        if reason == 'stop':
            value.stop()
        assert not value.assess(1_060_000_000, identity_verified=reason != 'identity')['valid']
        assert not value.assess(1_070_000_000, identity_verified=True)['valid']


def test_repeated_begin_cannot_restart_session():
    value = gate()
    assert value.begin(1_050_000_000, identity_verified=True)
    assert not value.begin(1_060_000_000, identity_verified=True)


def test_decoder_fault_cannot_be_cleared_by_new_clock():
    value = gate()
    value.status.fault = 'sample_regression'
    assert not value.begin(1_050_000_000, identity_verified=True)
    value.clock.request(1_100_000_000)
    assert value.ingest(reply(remote=5_100_000_000, token=1_100_000_000), 1_120_000_000)
    assert not value.begin(1_120_000_000, identity_verified=True)


def test_new_uncertain_safe_sample_uses_only_still_valid_previous_evidence():
    value = gate()
    assert value.begin(1_050_000_000, identity_verified=True)
    value.status.sample = sample(5100)
    value.status.sample['received'] = 1.06
    result = value.assess(1_060_000_000, identity_verified=True)
    assert result['valid'] and result['evidence_selection'] == 'previous_still_fresh'
    assert result['evidence_sample']['received'] == 1.05
    assert not result['latest_pending']['valid']
    assert value.proven_sample['stamp'] == 5005


def test_uncertain_latest_cannot_refresh_old_sample_expiry():
    value = gate()
    assert value.begin(1_050_000_000, identity_verified=True)
    value.clock.request(1_500_000_000)
    assert value.ingest(reply(remote=5_500_000_000, token=1_500_000_000), 1_540_000_000)
    value.status.sample = sample(5600)
    value.status.sample['received'] = 1.55
    assert not value.assess(1_550_000_000, identity_verified=True)['valid']
    assert value.fault


def test_unsafe_uncertain_latest_overrides_still_fresh_proof():
    value = gate()
    assert value.begin(1_050_000_000, identity_verified=True)
    value.status.sample = sample(5100)
    value.status.sample['bits'] |= 2
    value.status.sample['received'] = 1.06
    assert not value.assess(1_060_000_000, identity_verified=True)['valid']
    assert value.fault
