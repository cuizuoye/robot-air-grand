import pytest
from parameter_snapshot import ParameterSnapshot


def test_bounded_snapshot():
    s=ParameterSnapshot();s.begin(10,fresh=True,identity_ok=True)
    assert s.valid(29.99,identity_ok=True,scope_ok=True)
    assert not s.valid(30,identity_ok=True,scope_ok=True)
    assert not s.valid(11,identity_ok=True,scope_ok=True)


@pytest.mark.parametrize('identity,scope',[(False,True),(True,False)])
def test_fault_latches(identity,scope):
    s=ParameterSnapshot();s.begin(10,fresh=True,identity_ok=True)
    assert not s.valid(11,identity_ok=identity,scope_ok=scope)
    assert not s.valid(12,identity_ok=True,scope_ok=True)


@pytest.mark.parametrize('now,fresh,identity',[(0,False,True),(0,True,False),(float('nan'),True,True)])
def test_requires_fresh_configuration(now,fresh,identity):
    with pytest.raises(RuntimeError):ParameterSnapshot().begin(now,fresh=fresh,identity_ok=identity)


def test_single_use():
    s=ParameterSnapshot();s.begin(0,fresh=True,identity_ok=True)
    with pytest.raises(RuntimeError):s.begin(1,fresh=True,identity_ok=True)
