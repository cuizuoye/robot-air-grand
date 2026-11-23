from test_lock_clock_probe import ready, sample, reply
from lock_clock_probe import DeferredLockAudit


def queued():
    clock=ready();audit=DeferredLockAudit();value=sample(5050)
    detail=clock.assess(value,1_050_000_000,identity_verified=True)
    assert detail['reasons']==['source_age_uncertain']
    row=dict(detail=detail,accepted=True,locked_fresh=False)
    audit.add(row,value,1_050_000_000)
    return clock,audit,row,value


def test_no_new_reply_never_turns_waiting_into_proof():
    clock,audit,row,value=queued()
    audit.advance(clock,1_200_000_000,identity_verified=True,latest_sample=value)
    assert not row['locked_fresh'] and len(audit.pending)==1
    audit.advance(clock,1_450_000_001,identity_verified=True,latest_sample=value)
    assert row['deferred_failure']=='clock_recheck_timeout' and not audit.pending


def test_new_matching_reply_rechecks_original_sample_and_age():
    clock,audit,row,value=queued()
    clock.request(1_100_000_000)
    assert clock.ingest(reply(remote=5_100_000_000,token=1_100_000_000),1_120_000_000)
    audit.advance(clock,1_120_000_000,identity_verified=True,latest_sample=value)
    assert row['locked_fresh'] and not audit.pending
    assert row['initial_detail']['reasons']==['source_age_uncertain']
    assert row['detail']['source_age_bounds_ms'][0]>0
    assert row['deferred_verified_at_ns']==1_120_000_000


def test_unsafe_new_status_or_identity_loss_cannot_be_replaced_with_old_lock():
    for identity,bits in [(False,5),(True,7)]:
        clock,audit,row,value=queued()
        value['bits']=(value['bits'] & ~255)|bits
        audit.advance(clock,1_100_000_000,identity_verified=identity,latest_sample=value)
        assert not row['locked_fresh'] and not audit.pending
        assert row['deferred_failure'] in ('identity_not_fresh','unsafe_later_state')


def test_expired_and_other_failures_do_not_enter_queue():
    clock=ready();audit=DeferredLockAudit();value=sample(2000)
    row=dict(detail=clock.assess(value,1_050_000_000,identity_verified=True),accepted=True,locked_fresh=False)
    audit.add(row,value,1_050_000_000)
    assert not audit.pending


def test_finish_preserves_unresolved_failure():
    clock,audit,row,value=queued();audit.finish()
    assert not row['locked_fresh']
    assert row['deferred_failure']=='trial_ended_before_clock_proof'
