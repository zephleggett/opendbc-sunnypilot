"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car.interfaces import get_speed_dependent_torque_params


class CarInterfaceExt:
  """Speed-dependent torque callbacks for cars configured in speed_dependent.toml.

  Uses a quadratic LAF curve and power law friction model. The online
  learner provides per-bin SVD values; when all bins converge, the curve
  coefficients are refit from the learned values.
  """

  def __init__(self, CP, CI_Base):
    self.CP = CP
    self.CI_Base = CI_Base
    self.v_ego = 0.0

    speed_dep_params = get_speed_dependent_torque_params()
    cfg = speed_dep_params.get(CP.carFingerprint)

    if cfg is not None:
      self.speed_dep = True
      self.speed_dep_speed_bp = list(cfg['speed_bp'])
      # Quadratic LAF: LAF(v) = a*v^2 + b*v + c
      self._laf_a = cfg['laf_a']
      self._laf_b = cfg['laf_b']
      self._laf_c = cfg['laf_c']
      # Power law friction: friction(v) = a * v^b
      self._fric_a = cfg.get('fric_a', 0.1)
      self._fric_b = cfg.get('fric_b', 0.0)
      # Derive per-bin values from initial curves for learner tracking
      self.speed_dep_laf_v = [self._eval_laf(v) for v in self.speed_dep_speed_bp]
      self.speed_dep_friction_v = [self._eval_friction(v) for v in self.speed_dep_speed_bp]
      # Store originals for sanity bounds on live-learned updates
      self._original_laf_v = list(self.speed_dep_laf_v)
    else:
      self.speed_dep = False

  def _eval_laf(self, v):
    """Evaluate quadratic LAF curve: a*v^2 + b*v + c, floored at 0.5."""
    return max(self._laf_a * v**2 + self._laf_b * v + self._laf_c, 0.5)

  def _eval_friction(self, v):
    """Evaluate power law friction: a * v^b."""
    if v <= 0:
      return self._fric_a
    return self._fric_a * v ** self._fric_b

  def torque_from_lateral_accel_speed_dep_closure(self, lateral_acceleration, torque_params):
    """Upstream closure: torque = lat_accel / LAF(v_ego)."""
    return lateral_acceleration / self._eval_laf(self.v_ego)

  def lateral_accel_from_torque_speed_dep_closure(self, torque, torque_params):
    """Upstream closure (inverse): lat_accel = torque * LAF(v_ego)."""
    return torque * self._eval_laf(self.v_ego)

  def _torque_from_lateral_accel_speed_dep_torque_space(self, latcontrol_inputs, torque_params, gravity_adjusted):
    """SP torque-space callback: uses latcontrol_inputs.vego for speed."""
    return latcontrol_inputs.lateral_acceleration / self._eval_laf(latcontrol_inputs.vego)

  def torque_from_lateral_accel_in_torque_space(self):
    """Return the appropriate torque-space callback based on config."""
    if self.speed_dep:
      return self._torque_from_lateral_accel_speed_dep_torque_space
    return self.CI_Base.torque_from_lateral_accel_linear_in_torque_space

  def update_speed_dep_laf(self, speed_bp, laf_bp, friction_bp, valid_bp):
    """Apply live-learned per-bin values and refit curves when all bins valid.

    Per-bin LAF values are clamped to +/-30% of initial curve-derived values.
    Curve refit only triggers when ALL bins pass calibration.
    """
    n = len(self.speed_dep_laf_v)
    # Update per-bin LAF values with sanity bounds
    for i in range(n):
      if i < len(valid_bp) and valid_bp[i]:
        lo = self._original_laf_v[i] * 0.7
        hi = self._original_laf_v[i] * 1.3
        if lo <= laf_bp[i] <= hi:
          self.speed_dep_laf_v[i] = laf_bp[i]

    # Refit curves when ALL bins are valid
    if len(valid_bp) >= n and all(valid_bp[:n]):
      # Refit quadratic LAF from per-bin values
      new_coeffs = np.polyfit(self.speed_dep_speed_bp, self.speed_dep_laf_v, 2)
      # Sanity: curve minimum in driving range must be > 0.5
      v_min = -new_coeffs[1] / (2 * new_coeffs[0]) if new_coeffs[0] != 0 else 0
      laf_min = float(np.polyval(new_coeffs, np.clip(v_min, 3, 40)))
      if laf_min > 0.5:
        self._laf_a, self._laf_b, self._laf_c = float(new_coeffs[0]), float(new_coeffs[1]), float(new_coeffs[2])

      # Refit friction power law: log(friction) = log(a) + b*log(v)
      valid_frictions = [(s, f) for s, f, v in zip(speed_bp, friction_bp, valid_bp) if v and f > 0 and s > 0]
      if len(valid_frictions) >= 2:
        log_s = np.log([s for s, _ in valid_frictions])
        log_f = np.log([f for _, f in valid_frictions])
        b, log_a = np.polyfit(log_s, log_f, 1)
        self._fric_a, self._fric_b = float(np.exp(log_a)), float(b)
