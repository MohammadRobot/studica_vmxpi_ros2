from studica_robot_platform.manual_override import OverrideGate


def joy(g, now, held, x=1.0):
    return g.joy(now, now, [0, 0, 0, 0, held], [0., x, 0., 0.])


def test_manual_priority_release_and_new_goal_only():
    g = OverrideGate()
    g.goals([('old', 1., 2)], 1.)
    g.navigation(1., .1, .3)
    assert g.output(1., 1.) == (.1, .3)
    joy(g, 1.1, 0)
    assert joy(g, 1.2, 1)
    assert g.output(1.2, 1.2) == (.2, 0.)
    joy(g, 1.3, 0)
    g.goals([('old', 1., 2)], 1.4)
    g.navigation(1.4, .1, .3)
    assert g.output(1.4, 1.4) == (0., 0.)
    # A new goal while old cancellation is outstanding cannot resume motion.
    g.goals([('old', 1., 3), ('too_early', 1.5, 2)], 1.5)
    assert g.mode == 'WAITING_FOR_NEW_GOAL'
    g.goals([('old', 1., 5), ('new', 1.6, 2)], 1.6)
    assert g.output(1.6, 1.6) == (0., 0.)  # flush old nav velocity
    g.navigation(1.7, .1, .0)
    assert g.output(1.7, 1.7) == (.1, 0.)


def test_disconnect_needs_release_before_reconnect():
    g = OverrideGate()
    joy(g, 1., 0)
    joy(g, 1.1, 1)
    assert g.output(1.5, 1.5) == (0., 0.)
    joy(g, 1.6, 1)
    assert g.output(1.6, 1.6) == (0., 0.)
    joy(g, 1.7, 0)
    joy(g, 1.8, 1)
    assert g.output(1.8, 1.8) == (.2, 0.)


def test_malformed_and_nonfinite_commands_stop():
    g = OverrideGate()
    joy(g, 1., 0)
    joy(g, 1.1, 1)
    g.joy(1.2, 1.2, [], [])
    assert g.output(1.2, 1.2) == (0., 0.)
    g.goals([('new', 2., 2)], 2.)
    g.navigation(2., float('nan'), 0.)
    assert g.output(2., 2.) == (0., 0.)


def test_goal_during_takeover_cannot_resume_after_release():
    g = OverrideGate()
    joy(g, 1., 0)
    joy(g, 1.1, 1)
    g.goals([('during_manual', 1.2, 2)], 1.2)
    joy(g, 1.3, 0)
    g.goals([('during_manual', 1.2, 2)], 1.4)
    assert g.mode == 'WAITING_FOR_NEW_GOAL'


def test_held_on_startup_and_navigation_timeout():
    g = OverrideGate()
    joy(g, 1., 1)
    assert g.output(1., 1.) == (0., 0.)
    joy(g, 2., 0)
    g.goals([('new', 3., 2)], 3.)
    g.navigation(3., .1, .0)
    assert g.output(3.3, 3.3) == (0., 0.)
    g.navigation(3.4, .1, .0)
    assert g.output(3.4, 3.4) == (0., 0.)


def test_drive_turbo_preserves_deadman_and_disconnect_stop():
    g = OverrideGate(.2, .6, .3, .9)
    g.joy(1., 1., [0, 0, 0, 0, 0, 1], [0., 1., 0., 1.])
    assert g.output(1., 1.) == (0., 0.)
    g.joy(1.1, 1.1, [0, 0, 0, 0, 1, 1], [0., 1., 0., 1.])
    assert g.output(1.1, 1.1) == (.3, .9)
    assert g.output(1.4, 1.4) == (0., 0.)
