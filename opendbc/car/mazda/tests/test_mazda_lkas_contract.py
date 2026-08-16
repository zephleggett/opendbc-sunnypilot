#!/usr/bin/env python3
"""OEM CAM_LKAS / 0x440 contract from Route 3C vs failed Routes 34/36/39."""

from opendbc.can import CANPacker, CANParser
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.mazdacan import FSC_LNV1_LKAS_REQUEST_MAX
from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.car.structs import CarParams


def _lkas(lnv=1, err1=0, err2=0, bit1=1):
  return {
    "LINE_NOT_VISIBLE": lnv,
    "ERR_BIT_1": err1,
    "ERR_BIT_2": err2,
    "BIT_1": bit1,
  }


def _lane(tja=0, ll=1, lnv=1, vis=0, tr=0):
  return {
    "TJA": tja,
    "TJA_TRANSITION": tr,
    "LANE_LINES": ll,
    "LINE_NOT_VISIBLE": lnv,
    "LINE_VISIBLE": vis,
    "BIT1": 1,
    "BIT2": 0,
    "BIT3": 1,
    "NO_ERR_BIT": 0,
    "S1": 1,
    "S1_HBEAM": 0,
  }


def _cp():
  CP = CarParams()
  CP.flags = int(MazdaFlags.GEN1)
  CP.carFingerprint = str(CAR.MAZDA_CX5_2022)
  return CP


def _decode_243(dat):
  cp = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)
  cp.update([(0, [(0x243, dat, 0)])])
  return dict(cp.vl["CAM_LKAS"])


def _decode_440(dat):
  cp = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 0)
  cp.update([(0, [(0x440, dat, 0)])])
  return dict(cp.vl["CAM_LANEINFO"])


class TestFscEnvelope:
  def test_err_bits_block_steer(self):
    assert mazdacan.fsc_cam_lkas_allows_steer(_lkas(err1=1)) is False
    assert mazdacan.fsc_cam_lkas_allows_steer(_lkas(err2=1)) is False
    assert mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=1)) is True
    assert mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=0)) is True

  def test_lnv1_does_not_zero_request(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 0, 150, _lkas(lnv=1))
    v = _decode_243(msg[1])
    assert v["LINE_NOT_VISIBLE"] == 1
    assert v["LKAS_REQUEST"] == 150

  def test_lnv1_caps_route39_1088(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 13, 1088, _lkas(lnv=1))
    v = _decode_243(msg[1])
    assert v["LINE_NOT_VISIBLE"] == 1
    assert abs(v["LKAS_REQUEST"]) == FSC_LNV1_LKAS_REQUEST_MAX
    assert v["LKAS_REQUEST"] == FSC_LNV1_LKAS_REQUEST_MAX

  def test_lnv0_does_not_cap_at_200(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 3, 537, _lkas(lnv=0))
    v = _decode_243(msg[1])
    assert v["LINE_NOT_VISIBLE"] == 0
    assert v["LKAS_REQUEST"] == 537

  def test_err_zeros_even_if_lnv0(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 1, 400, _lkas(lnv=0, err1=1))
    v = _decode_243(msg[1])
    assert v["LKAS_REQUEST"] == 0
    assert v["ERR_BIT_1"] == 1


class TestHudTjaCoherence:
  def test_idle_copies_fsc_tja(self):
    assert mazdacan.hud_tja_for_request(_lane(tja=0), _lkas(lnv=1), 0) == 0
    assert mazdacan.hud_tja_for_request(_lane(tja=2), _lkas(lnv=1), 0) == 2

  def test_request_with_lnv0_is_tja4(self):
    assert mazdacan.hud_tja_for_request(_lane(tja=0), _lkas(lnv=0), 163) == 4
    assert mazdacan.hud_tja_for_request(_lane(tja=2), _lkas(lnv=0), 163) == 4

  def test_request_with_lnv1_is_tja3_not_tja4(self):
    assert mazdacan.hud_tja_for_request(_lane(tja=0), _lkas(lnv=1), 1088) == 3
    assert mazdacan.hud_tja_for_request(_lane(tja=2), _lkas(lnv=1), 12) == 3

  def test_never_tja4_without_request(self):
    assert mazdacan.hud_tja_for_request(_lane(tja=0), _lkas(lnv=0), 0) == 0

  def test_alert_command_route39_would_not_tx_tja0(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_alert_command(packer, _lane(tja=0, ll=1), False, False,
                                        apply_torque=1088, cam_lkas=_lkas(lnv=1))
    v = _decode_440(msg[1])
    assert v["TJA"] == 3
    assert v["LANE_LINES"] == 1

  def test_alert_command_oem_active_tja4(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_alert_command(packer, _lane(tja=2, ll=2, lnv=0, vis=1), False, False,
                                        apply_torque=163, cam_lkas=_lkas(lnv=0))
    v = _decode_440(msg[1])
    assert v["TJA"] == 4
    assert v["LANE_LINES"] == 2


class TestRoute39Replay:
  """What final TX would have been for Route 39's last accepted CAM_LKAS frames."""

  LAST_FRAMES = [
    (117.8346, 1040.0, 1, 0),
    (117.8442, 1052.0, 1, 0),
    (117.8538, 1064.0, 1, 0),
    (117.8646, 1076.0, 1, 0),
    (117.8745, 1088.0, 1, 0),
  ]

  def test_last_10s_contract(self):
    packer = CANPacker("mazda_2017")
    CP = _cp()
    rows = []
    for i, (t, old_req, old_lnv, old_tja) in enumerate(self.LAST_FRAMES):
      lkas = _lkas(lnv=old_lnv)
      steer = mazdacan.create_steering_control(packer, CP, i, int(old_req), lkas)
      hud = mazdacan.create_alert_command(packer, _lane(tja=old_tja), False, False,
                                          apply_torque=int(old_req), cam_lkas=lkas)
      v243 = _decode_243(steer[1])
      v440 = _decode_440(hud[1])
      rows.append({
        "t": t,
        "old_request": old_req,
        "new_request": v243["LKAS_REQUEST"],
        "old_lnv": old_lnv,
        "new_lnv": int(v243["LINE_NOT_VISIBLE"]),
        "old_tja": old_tja,
        "new_tja": int(v440["TJA"]),
      })
      assert v243["LINE_NOT_VISIBLE"] == old_lnv
      assert abs(v243["LKAS_REQUEST"]) <= FSC_LNV1_LKAS_REQUEST_MAX
      assert v440["TJA"] != 0
      assert v440["TJA"] == 3
      assert v243["ERR_BIT_1"] == 0
    assert rows[-1]["new_request"] == FSC_LNV1_LKAS_REQUEST_MAX
    assert rows[-1]["new_lnv"] == 1


class TestStateFuzz:
  def test_invariants(self):
    packer = CANPacker("mazda_2017")
    CP = _cp()
    for lnv in (0, 1):
      for err1 in (0, 1):
        for req in (0, 12, 177, 200, 537, 1088, -1088):
          for tja in (0, 2, 3, 4):
            lkas = _lkas(lnv=lnv, err1=err1)
            steer = mazdacan.create_steering_control(packer, CP, 5, req, lkas)
            hud = mazdacan.create_alert_command(packer, _lane(tja=tja), False, False,
                                                apply_torque=req, cam_lkas=lkas)
            v243 = _decode_243(steer[1])
            v440 = _decode_440(hud[1])
            wire = int(v243["LKAS_REQUEST"])
            if err1:
              assert wire == 0
            elif lnv and abs(req) > FSC_LNV1_LKAS_REQUEST_MAX:
              assert abs(wire) == FSC_LNV1_LKAS_REQUEST_MAX
            else:
              assert wire == req
            if abs(req) >= 1 and not err1:
              assert v440["TJA"] != 0
              if lnv:
                assert v440["TJA"] == 3
              else:
                assert v440["TJA"] == 4
            if lnv:
              assert v243["LINE_NOT_VISIBLE"] == 1
            else:
              assert v243["LINE_NOT_VISIBLE"] == 0
