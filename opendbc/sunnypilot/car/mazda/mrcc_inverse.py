"""MRCC inverse solver for Mazda CX-5 2022.

Given (current speed, target speed, distance to target), returns the smallest
ICBM set-speed command that brings the vehicle to v_target by the time it
covers d_to_target.

Two-stage forward model:
  Stage 1 (steady-state asymptote):  a_ss = mrcc_steady_decel(gap_mph)
  Stage 2 (transient dynamics):       da_ego/dt = (a_ss - a_ego) / TAU_RESPONSE_S
                                      with T_DEAD_S delay before any response
  Distance integration:                trapezoidal rule (exact for constant a
                                      over dt; second-order vs Euler).

Empirical sources (see tools/mazda_long/):
  - Steady-state table: /tmp/mrcc_gap_decel_v2.py, 150k+ samples,
    cruiseState.enabled, gap held >=1s, |pitch| < 0.02 rad,
    no driver inputs, no close lead, v_ego >= 5 m/s.
  - Transient dynamics: /tmp/mrcc_transient_v2.py +
    /tmp/mrcc_accel_cmd_xcorr_v2.py (277k cmd/a_ego pairs).
  - Validation: -3.55 mph mean prediction bias against 25 holdout SCC Vision
    events in device_data/0000019c-- and device_data/0000019e--.
"""
import math

import numpy as np

from opendbc.car.common.conversions import Conversions as CV

# === Stage 1: Steady-state asymptote ===
# Empirical median a_ego (m/s^2) by sustained gap (mph). Negative = decel.
_GAP_MPH = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20])
_A_EGO = np.array([0.0, -0.02, -0.02, -0.34, -0.44, -0.43, -0.40, -0.54, -0.66,
                   -0.69, -0.66, -0.55, -0.67, -0.66, -0.66, -0.84, -0.87, -0.85,
                   -0.82, -0.99])

# Conservative variant (p25 -- faster decel when MRCC commits hard).
_A_EGO_P25 = np.array([0.0, -0.10, -0.13, -0.51, -0.57, -0.56, -0.57, -0.68, -0.78,
                       -0.75, -0.75, -0.72, -0.80, -0.78, -0.79, -0.89, -0.97, -0.93,
                       -0.83, -1.06])

# === Stage 2: Transient dynamics ===
# Empirical: ~300ms dead time before vehicle responds to a gap step,
# followed by ~300ms first-order tau. 90% rise time ~700-900ms.
T_DEAD_S = 0.3
TAU_RESPONSE_S = 0.3

# === Mazda CX-5 MRCC settable speed floor (mph). Below this, the body ECU
# refuses to lower CRZ_SPEED further.
MIN_SET_SPEED_MPH = 19.0

# === Mazda long-press behavior (verified empirically from rlogs).
# A sustained CRZ_BTNS hold past ~0.5s triggers exactly a 5 mph step in
# CRZ_SPEED. Continuing to hold triggers additional 5-mph cycles ~0.5s apart.
LONG_PRESS_STEP_MPH = 5
LONG_PRESS_DURATION_S = 0.6
LONG_PRESS_COOLDOWN_S = 0.3


def mrcc_steady_decel(gap_mph, conservative=False):
    """Steady-state a_ego (m/s^2) for a sustained gap (mph).

    A gap of 0-2 mph is in MRCC's deadband -> returns 0.
    Beyond ~17 mph the response saturates; we cap at the last measured bin.
    NaN/non-finite inputs return 0 (defensive: prevents NaN propagation
    through forward_simulate -> inverse_solve -> ICBM state machine).
    """
    if not math.isfinite(gap_mph) or gap_mph <= 2.0:
        return 0.0
    # max/min on scalar avoids np.clip's array allocation in the hot loop.
    g = max(0.0, min(float(gap_mph), float(_GAP_MPH[-1])))
    table = _A_EGO_P25 if conservative else _A_EGO
    return float(np.interp(g, _GAP_MPH, table))


def forward_simulate(v_now_mph, sp_command_mph, d_total_m, dt=0.05,
                     conservative=False, a_ego_initial=0.0):
    """Integrate vehicle motion forward from (v_now, sp_command) until d_total
    is covered. Returns (v_at_target_mph, t_total_s, traj).

    Trapezoidal distance integration (exact for constant a over dt).
    Two-stage dynamics: T_DEAD_S delay then first-order lag to steady-state.

    Assumes sp_command is held constant during the approach (worst case for
    decel -- in practice ICBM would lift it as we close).
    """
    v = v_now_mph * CV.MPH_TO_MS
    sp = sp_command_mph * CV.MPH_TO_MS
    a_ego = float(a_ego_initial)
    d_remaining = float(d_total_m)
    t = 0.0
    traj = []
    while d_remaining > 0 and t < 30.0:
        gap_mph = max(0.0, (v - sp) / CV.MPH_TO_MS)
        a_target = mrcc_steady_decel(gap_mph, conservative)

        # Stage 2: dead time then first-order ramp
        if t >= T_DEAD_S:
            a_ego += (a_target - a_ego) * dt / TAU_RESPONSE_S

        # Trapezoidal velocity + distance update
        v_next = max(sp, v + a_ego * dt)
        d_step = 0.5 * (v + v_next) * dt
        d_remaining -= d_step
        traj.append((t, v / CV.MPH_TO_MS, a_ego, gap_mph))
        v = v_next
        t += dt
    return v / CV.MPH_TO_MS, t, traj


def inverse_solve(v_now_mph, v_target_mph, d_to_target_m,
                  conservative=False, max_overshoot_mph=18.0,
                  a_ego_initial=0.0):
    """Find the smallest sp_command (mph) that brings the vehicle to
    v_target by the time it covers d_to_target.

    Returns a dict:
      {
        'sp_command_mph': float,
        'achievable': bool,
        'predicted_v_at_target_mph': float,
        'time_to_target_s': float,
        'shortfall_mph': float (positive = won't make it),
      }

    When d_to_target_m <= 0 (no distance info), falls back to max-overshoot
    aggressive command so behavior degrades gracefully toward today's ICBM.
    """
    if v_target_mph >= v_now_mph:
        # No decel needed
        return {
            'sp_command_mph': v_now_mph,
            'achievable': True,
            'predicted_v_at_target_mph': v_now_mph,
            'time_to_target_s': d_to_target_m / max(v_now_mph * CV.MPH_TO_MS, 1.0) if d_to_target_m > 0 else 0.0,
            'shortfall_mph': 0.0,
        }

    sp_floor = max(MIN_SET_SPEED_MPH, v_target_mph - max_overshoot_mph)
    sp_ceiling = v_target_mph  # commanding above target wouldn't decel further

    # No distance budget: command max overshoot (matches today's ICBM).
    if d_to_target_m <= 0:
        return {
            'sp_command_mph': sp_floor,
            'achievable': True,
            'predicted_v_at_target_mph': v_target_mph,
            'time_to_target_s': 0.0,
            'shortfall_mph': 0.0,
        }

    # Probe max overshoot first to test achievability
    v_best, t_best, _ = forward_simulate(v_now_mph, sp_floor, d_to_target_m,
                                          conservative=conservative, a_ego_initial=a_ego_initial)
    if v_best > v_target_mph + 0.5:
        return {
            'sp_command_mph': sp_floor,
            'achievable': False,
            'predicted_v_at_target_mph': v_best,
            'time_to_target_s': t_best,
            'shortfall_mph': v_best - v_target_mph,
        }

    # Binary search for the gentlest sp that still hits v_target
    lo, hi = sp_floor, sp_ceiling
    for _ in range(20):
        mid = 0.5 * (lo + hi)
        v_mid, _, _ = forward_simulate(v_now_mph, mid, d_to_target_m,
                                        conservative=conservative, a_ego_initial=a_ego_initial)
        if v_mid <= v_target_mph:
            lo = mid  # this sp works; try larger (gentler)
        else:
            hi = mid  # too gentle; need lower sp
    sp_final = lo
    v_final, t_final, _ = forward_simulate(v_now_mph, sp_final, d_to_target_m,
                                            conservative=conservative, a_ego_initial=a_ego_initial)
    return {
        'sp_command_mph': sp_final,
        'achievable': True,
        'predicted_v_at_target_mph': v_final,
        'time_to_target_s': t_final,
        'shortfall_mph': max(0.0, v_final - v_target_mph),
    }


def split_long_short_presses(drop_mph):
    """For a desired CRZ_SPEED drop (mph), return (long_press_cycles, short_presses).

    Each long-press cycle drops 5 mph; each short press drops 1 mph.
    The remainder (0-4 mph) is handled by short presses to fine-tune.
    """
    drop_mph = max(0, int(round(drop_mph)))
    cycles = drop_mph // LONG_PRESS_STEP_MPH
    remainder = drop_mph - cycles * LONG_PRESS_STEP_MPH
    return cycles, remainder
