#!/usr/bin/env python3
"""Tests for Mazda CX-5 2022 steering parameters."""
from unittest.mock import Mock, patch

import numpy as np
import pytest

from opendbc.car import uds
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.longitudinal import (
  CAM_BUS,
  RADAR_ADDR,
  RADAR_BUS,
  build_crz_ctrl,
  build_crz_info,
  create_longitudinal_messages,
  create_radar_heartbeat_messages,
  request_radar_default_session,
)
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

  def test_inactive_available_crz_info_allows_acc_set(self):
    expected = [
      "01ffe20004800099",
      "01ffe20004800198",
      "01ffe20004800297",
      "01ffe20004800396",
      "01ffe20004800495",
      "01ffe20004800594",
      "01ffe20004800693",
      "01ffe20004800792",
      "01ffe20004800891",
      "01ffe20004800990",
      "01ffe20004800a8f",
      "01ffe20004800b8e",
      "01ffe20004800c8d",
      "01ffe20004800d8c",
      "01ffe20004800e8b",
      "01ffe20004800f8a",
    ]

    for counter, dat in enumerate(expected):
      assert build_crz_info(0.0, counter, False, False, 0.0, acc_set_allowed=True).hex() == dat

  def test_inactive_available_crz_ctrl_sets_available_state(self):
    assert build_crz_ctrl(False, False, False, False, crz_available=True).hex() == "02010b0000000000"

  @pytest.mark.parametrize(("crz_available", "expected"), [
    (False, [(0x21b, "01ffe3ffc000005d"), (0x21c, "0201010000000000")]),
    (True, [(0x21b, "01ffe20004800099"), (0x21c, "02010b0000000000")]),
  ])
  def test_inactive_longitudinal_pair_matches_crz_available(self, crz_available, expected):
    can_sends = create_longitudinal_messages(0, 0.0, 0, False, False, crz_available=crz_available)

    assert [(msg.address, msg.dat.hex()) for msg in can_sends] == expected

  def test_replacement_messages_can_target_radar_or_camera_bus(self):
    for bus in (RADAR_BUS, CAM_BUS):
      can_sends = [
        *create_longitudinal_messages(bus, 0.0, 0, False, False),
        *create_radar_heartbeat_messages(bus, 0),
      ]

      assert {msg.src for msg in can_sends} == {bus}

  def test_radar_heartbeat_matches_empty_stock_radar_tracks(self):
    expected = [
      (0x499, "0008c00000000000"),
      (0x361, "fff7fefe1fc00080"),
      (0x362, "fff7fefe1fc78c80"),
      (0x363, "fff7fefe1fc00000"),
      (0x364, "fff7fefe1fc00000"),
      (0x365, "fff7fe7ffbff3fc0"),
      (0x366, "fff7fe7ffbff3fc0"),
    ]

    can_sends = create_radar_heartbeat_messages(0, 0)
    assert [(msg.address, msg.dat.hex()) for msg in can_sends] == expected

  def test_radar_heartbeat_updates_track_counters(self):
    can_sends = create_radar_heartbeat_messages(0, 15)
    assert [msg.dat[-1] for msg in can_sends[1:]] == [0x8f, 0x8f, 0x0f, 0x0f, 0xcf, 0xcf]

  def test_radar_heartbeat_can_advertise_synthetic_lead_track(self):
    can_sends = create_radar_heartbeat_messages(0, 15, synthetic_lead=True)
    expected = [
      (0x499, "0008c00000000000"),
      (0x361, "fff7fefe1fc0008f"),
      (0x362, "fff7fefe1fc78c8f"),
      (0x363, "fff7fefe1fc0000f"),
      (0x364, "0a4000001dc0000f"),
      (0x365, "fff7fe7ffbff3fcf"),
      (0x366, "fff7fe7ffbff3fcf"),
    ]

    assert [(msg.address, msg.dat.hex()) for msg in can_sends] == expected

  def test_request_radar_default_session_uses_diagnostic_session_control(self):
    can_recv = Mock()
    can_send = Mock()
    request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.DEFAULT])
    response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.DEFAULT])

    with patch("opendbc.car.mazda.longitudinal._uds_request", return_value=True) as uds_request:
      assert request_radar_default_session(can_recv, can_send)

    uds_request.assert_called_once_with(can_recv, can_send, RADAR_BUS, RADAR_ADDR, request, response)

  @pytest.mark.parametrize(("openpilot_longitudinal", "expected_request"), [
    (False, False),
    (True, True),
  ])
  def test_deinit_requests_radar_default_session_for_longitudinal_control(self, openpilot_longitudinal, expected_request):
    class FakeCP:
      openpilotLongitudinalControl = openpilot_longitudinal

    can_recv = Mock()
    can_send = Mock()

    with patch("opendbc.car.mazda.interface.request_radar_default_session", return_value=True) as default_session:
      CarInterface.deinit(FakeCP, can_recv, can_send)

    if expected_request:
      default_session.assert_called_once_with(can_recv, can_send)
    else:
      default_session.assert_not_called()
