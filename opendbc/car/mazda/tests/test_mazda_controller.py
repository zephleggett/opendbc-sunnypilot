#!/usr/bin/env python3
"""Tests for Mazda CX-5 2022 steering parameters."""

import numpy as np
import pytest

from opendbc.car.mazda.longitudinal import build_crz_ctrl, build_crz_info
from opendbc.car.mazda.values import CAR, CarControllerParams


class TestCarControllerParams:

  @pytest.fixture
  def cx5_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5_2022
    return CarControllerParams(FakeCP())

  @pytest.fixture
  def pre_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5
    return CarControllerParams(FakeCP())

  def test_cx5_2022_has_lookup(self, cx5_2022_params):
    assert hasattr(cx5_2022_params, 'STEER_MAX_LOOKUP')
    assert cx5_2022_params.STEER_MAX == 1200

  def test_cx5_2022_low_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [0.0, 5.0, 10.0, 14.2]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 1200

  def test_cx5_2022_high_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [14.5, 20.0, 30.0]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 800

  def test_cx5_2022_rate_limits(self, cx5_2022_params):
    assert cx5_2022_params.STEER_DELTA_UP == 12
    assert cx5_2022_params.STEER_DELTA_DOWN == 25

  def test_cx5_2022_driver_multiplier_stock(self, cx5_2022_params):
    assert cx5_2022_params.STEER_DRIVER_MULTIPLIER == 15

  def test_pre_2022_no_lookup(self, pre_2022_params):
    assert not hasattr(pre_2022_params, 'STEER_MAX_LOOKUP')
    assert pre_2022_params.STEER_MAX == 800


class TestMazdaLongitudinalMessages:
  def test_inactive_crz_info_matches_stock_radar_standby(self):
    expected = [
      "01ffe3ffc000005d",
      "01ffe3ffc000015c",
      "01ffe3ffc000025b",
      "01ffe3ffc000035a",
      "01ffe3ffc0000459",
      "01ffe3ffc0000558",
      "01ffe3ffc0000657",
      "01ffe3ffc0000756",
      "01ffe3ffc0000855",
      "01ffe3ffc0000954",
      "01ffe3ffc0000a53",
      "01ffe3ffc0000b52",
      "01ffe3ffc0000c51",
      "01ffe3ffc0000d50",
      "01ffe3ffc0000e4f",
      "01ffe3ffc0000f4e",
    ]

    for counter, dat in enumerate(expected):
      assert build_crz_info(0.0, counter, False, False, 0.0).hex() == dat

  def test_inactive_crz_ctrl_matches_stock_radar_standby(self):
    assert build_crz_ctrl(False, False, False, False).hex() == "0201010000000000"
