import pytest
from h1_2_nav_tools.policy_motion_coordinator import MotionCoordinator, SessionToken

OWNER = 'a' * 32

def prepare(c, source='manual'):
    return c.begin_prepare(source, c.token)

def finish(c, op, **overrides):
    barriers = dict(isolated=True, zero_barrier=True, node_ready=True, backend_ready=True)
    barriers.update(overrides)
    return c.finish_prepare(op, **barriers)

def test_hold_wins_and_rejects_old_goal():
    c = MotionCoordinator(OWNER)
    p = prepare(c)
    assert finish(c, p.operation).success
    assert c.activate(p.token).success
    assert c.accepts(p.token)
    new = c.hold()
    assert new.epoch == p.token.epoch + 1
    assert not c.accepts(p.token)
    assert not c.activate(p.token).success

@pytest.mark.parametrize('barrier', ['isolated', 'zero_barrier'])
def test_all_barriers_required(barrier):
    c = MotionCoordinator(OWNER)
    p = prepare(c)
    assert not finish(c, p.operation, **{barrier: False}).success
    assert not c.activate(p.token).success


def test_interfaces_exposed_only_after_ready_and_no_replay():
    c = MotionCoordinator(OWNER)
    p = prepare(c)
    assert p.success and not p.cmd_topic and not p.action_namespace
    assert not c.activate(p.token).success
    result = finish(c, p.operation)
    assert result.cmd_topic.endswith('/cmd_vel')
    assert result.action_namespace.endswith('/frontend')
    assert c.activate(p.token).success
    assert not c.activate(p.token).success


def test_full_token_cas_and_restart():
    c = MotionCoordinator(OWNER)
    p = prepare(c)
    assert finish(c, p.operation).success
    for token in (SessionToken('b' * 32, p.token.coordinator_boot_id, p.token.epoch),
                  SessionToken(OWNER, 'b' * 32, p.token.epoch),
                  SessionToken(OWNER, p.token.coordinator_boot_id, True)):
        assert not c.activate(token).success
        assert not c.begin_prepare('manual', token).success
    other = MotionCoordinator(OWNER)
    assert other.token.coordinator_boot_id != c.token.coordinator_boot_id
    assert not other.begin_prepare('manual', c.token).success


def test_stale_finish_cannot_mutate_hold_or_new_prepare():
    c = MotionCoordinator(OWNER)
    first = prepare(c)
    c.hold()
    result = finish(c, first.operation)
    assert result.reason == 'STALE_PREPARE' and not result.cmd_topic
    assert c.state == 'hold'
    first = prepare(c)
    second = prepare(c)
    stale = finish(c, first.operation, isolated=False)
    assert stale.reason == 'STALE_PREPARE' and not stale.action_namespace
    assert c.state == 'switching' and c.token == second.token
    assert finish(c, second.operation).success
    assert c.activate(second.token).success
    assert finish(c, first.operation).reason == 'STALE_PREPARE'
    assert c.accepts(second.token)

@pytest.mark.parametrize('owner', ['A' * 32, '../' + 'a' * 29, 'a' * 31, 'é' * 32, None])
def test_owner_validation(owner):
    with pytest.raises((ValueError, TypeError)):
        MotionCoordinator(owner)


def test_nav_and_input_validation():
    c = MotionCoordinator(OWNER)
    assert not prepare(c, 'nav').success
    assert not prepare(c, 'other').success
    assert not c.begin_prepare('manual', None).success
    called = []
    with pytest.raises(TypeError):
        MotionCoordinator(OWNER, lambda: called.append(True))
    assert not called
    c = MotionCoordinator(OWNER, True)
    p = prepare(c, 'nav')
    assert p.success and finish(c, p.operation).success
    assert c.activate(p.token).success and c.accepts(p.token)


def test_backend_can_start_after_prepare():
    c = MotionCoordinator(OWNER)
    p = prepare(c, 'manual')
    ready = finish(c, p.operation, backend_ready=False)
    assert ready.success and ready.cmd_topic and ready.action_namespace
    assert not c.activate(p.token).success
    assert c.activate(p.token, backend_ready=True).success


def test_explicit_node_ready_required_at_activation():
    c = MotionCoordinator(OWNER)
    p = prepare(c)
    assert finish(c, p.operation, node_ready=False).success
    assert not c.activate(p.token).success
    assert not c.activate(p.token, node_ready=1).success
    assert c.activate(p.token, node_ready=True).success
