#!/usr/bin/env python3
"""Route 3C OEM active-TJA handshake contract vs failed Routes 34/36/39."""

from opendbc.can import CANPacker, CANParser
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.mazdacan import (
  FSC_LNV1_LKAS_REQUEST_MAX, LKAS_TX_ACTIVE, LKAS_TX_AUTH_PAUSED, LKAS_TX_FAULT,
  LKAS_TX_IDLE, LKAS_TX_TRANSITION_TO_ACTIVE, TJA3_ACTIVATION_HOLD_NS,
)
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


def _step(lat_active, now_ns, start_ns=None, prev=LKAS_TX_IDLE, desired=163,
          fsc_lnv=1, fsc_tja=0, err1=0, fsc_ll=1):
  return mazdacan.lkas_tx_step(
    lat_active=lat_active, fsc_ok=err1 == 0, now_ns=now_ns,
    handshake_start_ns=start_ns, prev_state=prev, desired_torque=desired,
    fsc_lkas=_lkas(lnv=fsc_lnv, err1=err1), fsc_lane=_lane(tja=fsc_tja, ll=fsc_ll, lnv=1),
  )


def _pack(tx, req=None):
  packer = CANPacker("mazda_2017")
  torque = tx.apply_torque if req is None else req
  steer = mazdacan.create_steering_control(packer, _cp(), 3, torque, _lkas(lnv=1),
                                           tx_lnv=tx.cam_lkas_lnv)
  hud = mazdacan.create_alert_command(packer, _lane(tja=0, ll=1, lnv=1), False, False, tx=tx)
  return _decode_243(steer[1]), _decode_440(hud[1])


class TestHandshakeStateMachine:
  def test_idle_copies_fsc(self):
    tx = _step(False, 0, fsc_tja=2, fsc_lnv=1, desired=400, fsc_ll=0)
    assert tx.state == LKAS_TX_IDLE
    assert tx.apply_torque == 0
    assert tx.tja == 2
    assert tx.cam_lkas_lnv == 1
    assert tx.lane_lines == 0
    assert tx.send_hud_every_frame is False

  def test_no_torque_before_auth(self):
    tx = _step(False, 0, desired=1088, fsc_lnv=1)
    assert tx.apply_torque == 0
    v243, v440 = _pack(tx)
    assert v243["LKAS_REQUEST"] == 0
    assert v440["TJA"] == 0

  def test_auth_enters_tja3_transition(self):
    tx = _step(True, 0, start_ns=None, desired=400, fsc_lnv=1, fsc_tja=0)
    assert tx.state == LKAS_TX_TRANSITION_TO_ACTIVE
    assert tx.tja == 3
    assert tx.cam_lkas_lnv == 0
    assert tx.apply_torque == 0
    assert tx.lane_lines == 1
    assert tx.line_visible == 1
    assert tx.hud_lnv == 0
    v243, v440 = _pack(tx)
    assert v243["LKAS_REQUEST"] == 0
    assert v243["LINE_NOT_VISIBLE"] == 0
    assert v440["TJA"] == 3
    assert v440["LANE_LINES"] == 1

  def test_transition_lasts_50ms_not_forever(self):
    t0 = 1_000_000_000
    tx0 = _step(True, t0, None, desired=400, fsc_lnv=1)
    tx_mid = _step(True, t0 + TJA3_ACTIVATION_HOLD_NS - 1, tx0.handshake_start_ns,
                   tx0.state, desired=400, fsc_lnv=1)
    tx_go = _step(True, t0 + TJA3_ACTIVATION_HOLD_NS, tx0.handshake_start_ns,
                  tx_mid.state, desired=400, fsc_lnv=1)
    assert tx_mid.state == LKAS_TX_TRANSITION_TO_ACTIVE
    assert tx_mid.apply_torque == 0
    assert tx_go.state == LKAS_TX_ACTIVE
    assert tx_go.tja == 4
    assert tx_go.cam_lkas_lnv == 0
    assert tx_go.apply_torque == 400

  def test_fsc_lnv1_does_not_hold_tja3(self):
    t0 = 0
    tx = _step(True, t0, None, desired=90, fsc_lnv=1, fsc_tja=0)
    for dt in (50_000_000, 2_000_000_000, 10_000_000_000):
      tx = _step(True, t0 + dt, tx.handshake_start_ns, tx.state, desired=90, fsc_lnv=1)
      assert tx.state == LKAS_TX_ACTIVE
      assert tx.tja == 4
      assert tx.cam_lkas_lnv == 0
      assert tx.apply_torque == 90

  def test_active_uses_oem_envelope(self):
    tx = _step(True, TJA3_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=537, fsc_lnv=1, fsc_tja=0)
    v243, v440 = _pack(tx)
    assert v243["LINE_NOT_VISIBLE"] == 0
    assert v243["LKAS_REQUEST"] == 537
    assert v243["ERR_BIT_1"] == 0
    assert v243["BIT_1"] == 1
    assert v243["LDW"] == 0
    assert v243["ANGLE_ENABLED"] == 0
    assert v243["STEERING_ANGLE"] == 0
    assert v440["TJA"] == 4
    assert v440["LANE_LINES"] == 1
    assert v440["LINE_VISIBLE"] == 0
    assert v440["LINE_NOT_VISIBLE"] == 1

  def test_active_does_not_clamp_to_200(self):
    tx = _step(True, TJA3_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=1088, fsc_lnv=1)
    v243, _ = _pack(tx)
    assert v243["LKAS_REQUEST"] == 1088
    assert v243["LINE_NOT_VISIBLE"] == 0

  def test_auth_loss_zeros_request_keeps_resume_path(self):
    active = _step(True, TJA3_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
                   desired=200, fsc_lnv=1)
    paused = _step(False, TJA3_ACTIVATION_HOLD_NS + 10_000_000, active.handshake_start_ns,
                   active.state, desired=200, fsc_lnv=1, fsc_tja=2)
    assert paused.state == LKAS_TX_AUTH_PAUSED
    assert paused.apply_torque == 0
    assert paused.tja == 2
    assert paused.handshake_start_ns is None

  def test_auth_restore_reenters_transition_then_active(self):
    paused = _step(False, 100, None, LKAS_TX_ACTIVE, desired=200, fsc_tja=2)
    t1 = 200
    tr = _step(True, t1, paused.handshake_start_ns, paused.state, desired=200, fsc_lnv=1)
    assert tr.state == LKAS_TX_TRANSITION_TO_ACTIVE
    assert tr.apply_torque == 0
    assert tr.tja == 3
    act = _step(True, t1 + TJA3_ACTIVATION_HOLD_NS, tr.handshake_start_ns, tr.state,
                desired=200, fsc_lnv=1)
    assert act.state == LKAS_TX_ACTIVE
    assert act.tja == 4
    assert act.apply_torque == 200

  def test_err_zeros_request(self):
    tx = _step(True, TJA3_ACTIVATION_HOLD_NS, 0, LKAS_TX_ACTIVE, desired=400, err1=1)
    assert tx.state == LKAS_TX_FAULT
    assert tx.apply_torque == 0
    packer = CANPacker("mazda_2017")
    steer = mazdacan.create_steering_control(packer, _cp(), 3, tx.apply_torque,
                                             _lkas(lnv=1, err1=1), tx_lnv=tx.cam_lkas_lnv)
    v243 = _decode_243(steer[1])
    assert v243["LKAS_REQUEST"] == 0
    assert v243["ERR_BIT_1"] == 1


class TestPackerBackstops:
  def test_err_zeros_even_if_lnv0(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 1, 400, _lkas(lnv=0, err1=1), tx_lnv=0)
    v = _decode_243(msg[1])
    assert v["LKAS_REQUEST"] == 0
    assert v["ERR_BIT_1"] == 1

  def test_lnv1_wire_still_caps(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 13, 1088, _lkas(lnv=1), tx_lnv=1)
    v = _decode_243(msg[1])
    assert v["LINE_NOT_VISIBLE"] == 1
    assert abs(v["LKAS_REQUEST"]) == FSC_LNV1_LKAS_REQUEST_MAX


class TestRouteReplay:
  """One state machine through Routes 34/36/39. latActive includes panda auth."""

  def _replay(self, t_auth, t_fault, fsc_lnv, desired=90):
    start = None
    prev = LKAS_TX_IDLE
    rows = []
    reached_active = False
    stayed_tja3 = False
    pre_auth_nz = False
    tja0_while_req = False
    t0 = int((t_auth - 0.05) * 1e9)
    t1 = int((t_fault + 0.05) * 1e9)
    for ns in range(t0, t1, 10_000_000):
      t = ns / 1e9
      lat = t >= t_auth
      err = 1 if t >= t_fault else 0
      tx = _step(lat, ns, start, prev, desired=desired, fsc_lnv=fsc_lnv, fsc_tja=0, err1=err)
      start, prev = tx.handshake_start_ns, tx.state
      if tx.state == LKAS_TX_ACTIVE:
        reached_active = True
      if t >= t_auth + 0.050 and t < t_fault and tx.state == LKAS_TX_TRANSITION_TO_ACTIVE:
        stayed_tja3 = True
      if t < t_auth and tx.apply_torque != 0:
        pre_auth_nz = True
      if tx.apply_torque != 0 and tx.tja == 0:
        tja0_while_req = True
      rows.append((round(t, 3), tx.state, tx.tja, tx.cam_lkas_lnv, tx.lane_lines,
                   tx.apply_torque, lat, err))
    return {
      "reached_active": reached_active,
      "stayed_tja3": stayed_tja3,
      "pre_auth_nz": pre_auth_nz,
      "tja0_while_req": tja0_while_req,
      "rows": rows,
    }

  def test_route39(self):
    r = self._replay(112.021, 117.882, fsc_lnv=1, desired=1088)
    assert r["pre_auth_nz"] is False
    assert r["tja0_while_req"] is False
    assert r["stayed_tja3"] is False
    assert r["reached_active"] is True
    active = [row for row in r["rows"] if row[1] == LKAS_TX_ACTIVE]
    assert active
    assert all(row[2] == 4 and row[3] == 0 and row[4] == 1 for row in active)
    trans = [row for row in r["rows"] if row[1] == LKAS_TX_TRANSITION_TO_ACTIVE]
    assert trans
    assert all(row[2] == 3 and row[5] == 0 for row in trans)

  def test_route36(self):
    r = self._replay(39.039, 62.199, fsc_lnv=1, desired=90)
    assert r["pre_auth_nz"] is False
    assert r["stayed_tja3"] is False
    assert r["reached_active"] is True
    assert r["tja0_while_req"] is False
    # Route 36 EPS EFFECTIVE with LANE_LINES=1: LL=2 is not required for apply.
    active = [row for row in r["rows"] if row[1] == LKAS_TX_ACTIVE]
    assert active
    assert all(row[2] == 4 and row[3] == 0 and row[4] == 1 and row[5] != 0 for row in active)

  def test_route34(self):
    r = self._replay(55.303, 61.213, fsc_lnv=1, desired=238)
    assert r["pre_auth_nz"] is False
    assert r["stayed_tja3"] is False
    assert r["reached_active"] is True
    assert r["tja0_while_req"] is False


class TestGoldenEnvelope:
  def test_active_matches_route3c_tja4(self):
    # Route 3C OEM-active correlated with LANE_LINES=2; copy that FSC value.
    tx = _step(True, TJA3_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=163, fsc_lnv=1, fsc_ll=2)
    _, v440 = _pack(tx)
    assert v440["TJA"] == 4
    assert v440["LANE_LINES"] == 2
    assert v440["LINE_NOT_VISIBLE"] == 1
    assert v440["LINE_VISIBLE"] == 0

  def test_transition_matches_route3c_tja3(self):
    tx = _step(True, 0, None, desired=163, fsc_lnv=1, fsc_ll=2)
    _, v440 = _pack(tx)
    assert v440["TJA"] == 3
    assert v440["LANE_LINES"] == 2
    assert v440["LINE_VISIBLE"] == 1
    assert v440["LINE_NOT_VISIBLE"] == 0

  def test_route3c_ll2_is_copied_not_forced(self):
    tx = _step(True, TJA3_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=163, fsc_lnv=1, fsc_ll=1)
    _, v440 = _pack(tx)
    assert v440["TJA"] == 4
    assert v440["LANE_LINES"] == 1


class TestHudLaneLinesCopy:
  def _active(self, fsc_ll, desired=400):
    return _step(True, TJA3_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
                 desired=desired, fsc_lnv=1, fsc_ll=fsc_ll)

  def test_a_active_fsc_ll0_copies_and_steers(self):
    tx = self._active(0)
    _, v440 = _pack(tx)
    assert tx.state == LKAS_TX_ACTIVE
    assert v440["TJA"] == 4
    assert v440["LANE_LINES"] == 0
    assert tx.apply_torque == 400
    assert tx.cam_lkas_lnv == 0

  def test_b_active_fsc_ll1_copies_and_steers(self):
    tx = self._active(1)
    _, v440 = _pack(tx)
    assert tx.state == LKAS_TX_ACTIVE
    assert v440["TJA"] == 4
    assert v440["LANE_LINES"] == 1
    assert tx.apply_torque == 400
    assert tx.cam_lkas_lnv == 0

  def test_c_active_fsc_ll2_copies_and_steers(self):
    tx = self._active(2)
    _, v440 = _pack(tx)
    assert tx.state == LKAS_TX_ACTIVE
    assert v440["TJA"] == 4
    assert v440["LANE_LINES"] == 2
    assert tx.apply_torque == 400
    assert tx.cam_lkas_lnv == 0

  def test_d_ll_change_while_active_is_hud_only(self):
    t0 = TJA3_ACTIVATION_HOLD_NS
    start = 0
    prev = LKAS_TX_TRANSITION_TO_ACTIVE
    seen = []
    for i, ll in enumerate((0, 1, 2, 0, 2, 1)):
      tx = _step(True, t0 + i * 10_000_000, start, prev, desired=537, fsc_lnv=1, fsc_ll=ll)
      start, prev = tx.handshake_start_ns, tx.state
      _, v440 = _pack(tx)
      assert tx.state == LKAS_TX_ACTIVE
      assert v440["TJA"] == 4
      assert v440["LANE_LINES"] == ll
      assert tx.apply_torque == 537
      assert tx.cam_lkas_lnv == 0
      seen.append(ll)
    assert seen == [0, 1, 2, 0, 2, 1]

  def test_e_tja4_independent_of_lane_lines(self):
    for ll in (0, 1, 2, 3):
      tx = self._active(ll, desired=90)
      assert tx.state == LKAS_TX_ACTIVE
      assert tx.tja == 4
      assert tx.lane_lines == ll


class TestStateFuzz:
  def test_invariants(self):
    packer = CANPacker("mazda_2017")
    CP = _cp()
    for lat in (False, True):
      for fsc_lnv in (0, 1):
        for err1 in (0, 1):
          for desired in (0, 12, 177, 200, 537, 1088, -1088):
            for fsc_tja in (0, 2, 3, 4):
              for fsc_ll in (0, 1, 2):
                for elapsed in (0, TJA3_ACTIVATION_HOLD_NS):
                  start = 0 if lat and err1 == 0 else None
                  tx = _step(lat, elapsed, start, LKAS_TX_IDLE if start is None else LKAS_TX_TRANSITION_TO_ACTIVE,
                             desired=desired, fsc_lnv=fsc_lnv, fsc_tja=fsc_tja, err1=err1, fsc_ll=fsc_ll)
                  steer = mazdacan.create_steering_control(packer, CP, 5, tx.apply_torque,
                                                           _lkas(lnv=fsc_lnv, err1=err1),
                                                           tx_lnv=tx.cam_lkas_lnv)
                  hud = mazdacan.create_alert_command(packer, _lane(tja=fsc_tja, ll=fsc_ll), False, False, tx=tx)
                  v243 = _decode_243(steer[1])
                  v440 = _decode_440(hud[1])
                  wire = int(v243["LKAS_REQUEST"])
                  assert tx.lane_lines == fsc_ll
                  assert v440["LANE_LINES"] == fsc_ll
                  if err1 or not lat:
                    assert wire == 0
                  if tx.state == LKAS_TX_TRANSITION_TO_ACTIVE:
                    assert wire == 0
                    assert v440["TJA"] == 3
                    assert tx.state != LKAS_TX_ACTIVE or elapsed >= TJA3_ACTIVATION_HOLD_NS
                  if tx.state == LKAS_TX_ACTIVE:
                    assert v440["TJA"] == 4
                    assert v243["LINE_NOT_VISIBLE"] == 0
                  if wire != 0:
                    assert v440["TJA"] != 0
                    assert v440["TJA"] != 3
                  if err1:
                    assert tx.state == LKAS_TX_FAULT
