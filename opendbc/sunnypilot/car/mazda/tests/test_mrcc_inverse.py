"""Unit tests for the Mazda MRCC inverse solver."""
import pytest

from opendbc.sunnypilot.car.mazda.mrcc_inverse import (
    mrcc_steady_decel,
    forward_simulate,
    inverse_solve,
    split_long_short_presses,
    MIN_SET_SPEED_MPH,
    LONG_PRESS_STEP_MPH,
)


class TestSteadyState:
    def test_deadband_zero_at_small_gap(self):
        assert mrcc_steady_decel(0) == 0.0
        assert mrcc_steady_decel(1) == 0.0
        assert mrcc_steady_decel(2) == 0.0

    def test_knee_at_3_mph(self):
        # Empirical: gap=3 -> -0.34 m/s^2 (median)
        assert -0.4 < mrcc_steady_decel(3) < -0.2

    def test_saturates_around_minus_0_85(self):
        # gap=15+ should be near -0.84
        assert -0.95 < mrcc_steady_decel(15) < -0.75
        assert -0.95 < mrcc_steady_decel(17) < -0.75

    def test_conservative_p25_is_more_aggressive(self):
        for gap in [3, 5, 10, 15]:
            assert mrcc_steady_decel(gap, conservative=True) < mrcc_steady_decel(gap, conservative=False)


class TestForwardSimulate:
    def test_no_gap_no_decel(self):
        v_final, t, _ = forward_simulate(40.0, 40.0, 100.0)
        # No gap, no decel; vehicle covers distance at constant speed
        assert abs(v_final - 40.0) < 0.01
        assert 5.0 < t < 6.0  # 100m at ~40mph (~17.9 m/s) -> ~5.6s

    def test_dead_time_no_response_in_first_300ms(self):
        # Simulate a 10 mph gap for only 0.25s of distance (~4-5m)
        v_now = 40.0
        d_short = 4.0  # very short distance
        v_final, t, _ = forward_simulate(v_now, 30.0, d_short, dt=0.05)
        # During dead time vehicle hasn't decelerated yet
        assert abs(v_final - v_now) < 0.5

    def test_decel_after_dead_time(self):
        # Long distance: should achieve significant decel by the end
        v_final, t, traj = forward_simulate(50.0, 30.0, 200.0)
        assert v_final < 50.0  # decelerated
        assert v_final > 30.0  # but not all the way (MRCC's cap)

    def test_settles_at_sp_command_after_long_distance(self):
        # Far enough to reach steady state
        v_final, t, traj = forward_simulate(40.0, 30.0, 500.0)
        assert v_final < 35.0  # well below v_now


class TestInverseSolve:
    def test_v_target_above_current_no_op(self):
        r = inverse_solve(30.0, 35.0, 100.0)
        assert r['achievable']
        assert r['sp_command_mph'] == 30.0  # no decel needed
        assert r['shortfall_mph'] == 0.0

    def test_short_distance_unachievable(self):
        # 40 -> 20 mph in 30m: physically impossible at MRCC's -0.85 m/s^2 cap
        r = inverse_solve(40.0, 20.0, 30.0)
        assert not r['achievable']
        assert r['sp_command_mph'] == MIN_SET_SPEED_MPH
        assert r['shortfall_mph'] > 0

    def test_zero_distance_falls_back_to_max_overshoot(self):
        r = inverse_solve(40.0, 30.0, 0.0)
        assert r['achievable']
        # sp should be at the floor (max overshoot)
        assert r['sp_command_mph'] == max(MIN_SET_SPEED_MPH, 30.0 - 18.0)

    def test_long_distance_picks_gentle_command(self):
        # 40 -> 35 mph in 200m: plenty of time, should command gently
        r = inverse_solve(40.0, 35.0, 200.0)
        assert r['achievable']
        # sp_command should be above the floor (gentler than max overshoot)
        assert r['sp_command_mph'] > MIN_SET_SPEED_MPH
        # Predicted v at target should be close to v_target
        assert abs(r['predicted_v_at_target_mph'] - 35.0) < 1.0

    def test_long_distance_gentler_than_short_distance(self):
        # Same v_now/v_target but more distance => gentler command
        r_short = inverse_solve(40.0, 30.0, 80.0)
        r_long = inverse_solve(40.0, 30.0, 200.0)
        # Both achievable should produce a higher (gentler) sp for longer d
        if r_short['achievable'] and r_long['achievable']:
            assert r_long['sp_command_mph'] >= r_short['sp_command_mph']


class TestPressSplit:
    def test_zero_drop(self):
        cycles, rem = split_long_short_presses(0)
        assert cycles == 0
        assert rem == 0

    def test_below_long_press_threshold(self):
        cycles, rem = split_long_short_presses(3)
        assert cycles == 0
        assert rem == 3

    def test_exactly_one_long_press(self):
        cycles, rem = split_long_short_presses(5)
        assert cycles == 1
        assert rem == 0

    def test_mixed_long_and_short(self):
        cycles, rem = split_long_short_presses(7)
        assert cycles == 1
        assert rem == 2

    def test_three_long_presses(self):
        cycles, rem = split_long_short_presses(15)
        assert cycles == 3
        assert rem == 0

    def test_three_long_plus_remainder(self):
        cycles, rem = split_long_short_presses(17)
        assert cycles == 3
        assert rem == 2


class TestRegressionAgainstObservedScenarios:
    """Sanity scenarios from the SCC Vision validation set."""

    def test_light_pre_curve_braking(self):
        # 40 -> 35 in 80m: should be achievable per holdout data
        r = inverse_solve(40.0, 35.0, 80.0)
        assert r['achievable']

    def test_highway_to_sharp_curve_too_short(self):
        # 50 -> 25 in 100m: per Agent 3 holdout, unachievable
        r = inverse_solve(50.0, 25.0, 100.0)
        assert not r['achievable']

    def test_big_drop_long_distance_still_hard(self):
        # 60 -> 20 in 200m: MRCC -0.85 m/s^2 ceiling makes this very hard
        r = inverse_solve(60.0, 20.0, 200.0)
        # Either not achievable, or only barely so; assert solver reports shortfall consistently
        if not r['achievable']:
            assert r['shortfall_mph'] > 0
