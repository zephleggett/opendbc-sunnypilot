#!/usr/bin/env python3
"""CAM_LKAS LINE_NOT_VISIBLE must stay coherent with FSC.

Comma used to hard-force LINE_NOT_VISIBLE=0 while requesting torque. On the
CX-5 2025 routes that latched an LKAS fault, FSC CAM_LKAS LNV stayed 1 and
FSC LKAS_REQUEST stayed 0. Production now copies FSC LNV and will not put a
nonzero LKAS_REQUEST on the wire against LNV=1 or ERR_BIT_*.
"""

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

# bus2 FSC CAM_LKAS captured on ff7df7d6f9c3403b|00000036--dca9bb4a70
# just before the FSC ERR_BIT rise (LNV=1, ERR=0, request=0)
ROUTE36_FSC_PREFAULT = bytes.fromhex("48000820020000c3")
# same route, first FSC ERR_BIT_1/2 frame
ROUTE36_FSC_FAULT = bytes.fromhex("68000960020000b0")


def _cp():
  fingerprint = {0: {}, 1: {}, 2: {}}
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long=False, is_release_sp=False, docs=False)
  return CP, CP_SP


def _lkas(*, lnv=0, err1=0, err2=0, bit1=1):
  return {
    "BIT_1": bit1,
    "ERR_BIT_1": err1,
    "ERR_BIT_2": err2,
    "LINE_NOT_VISIBLE": lnv,
  }


def _decode_cam_lkas(dat):
  cp = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)
  cp.update([(0, [(0x243, bytes(dat), 0)])])
  return cp.vl["CAM_LKAS"]


def _pack(CP, frame, torque, lkas):
  packer = CANPacker("mazda_2017")
  return mazdacan.create_steering_control(packer, CP, frame, torque, lkas)


def _checksum(ctr, torque, lnv, er1, er2, b1):
  tmp = torque + 2048
  lo = tmp & 0xFF
  hi = tmp >> 8
  steering_angle = 0
  b2 = 0
  ldw = 0
  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)
  csum = csum - ahi - amd - alo - b2
  if ahi == 1:
    csum = csum + 15
  if csum < 0:
    csum = csum + 512 if csum < -256 else csum + 256
  return csum % 256


class TestFscCamLkasAllowsSteer:
  def test_lnv0_no_err_allows(self):
    assert mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=0))

  def test_lnv1_blocks(self):
    assert not mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=1))

  def test_err_bits_block(self):
    assert not mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=0, err1=1))
    assert not mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=0, err2=1))

  def test_missing_lnv_defaults_blocked(self):
    assert not mazdacan.fsc_cam_lkas_allows_steer({"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 0})


class TestCreateSteeringControlCoherence:
  def setup_method(self):
    self.CP, _ = _cp()

  def test_lnv0_passes_through_torque(self):
    addr, dat, bus = _pack(self.CP, 4, 90, _lkas(lnv=0))
    assert addr == 0x243 and bus == 0
    v = _decode_cam_lkas(dat)
    assert int(v["LINE_NOT_VISIBLE"]) == 0
    assert int(v["LKAS_REQUEST"]) == 90
    assert int(v["CTR"]) == 4
    assert int(v["ERR_BIT_1"]) == 0
    assert int(v["CHKSUM"]) == _checksum(4, 90, 0, 0, 0, 1)

  def test_lnv1_copies_fsc_and_zeros_request(self):
    addr, dat, bus = _pack(self.CP, 4, 90, _lkas(lnv=1))
    v = _decode_cam_lkas(dat)
    assert int(v["LINE_NOT_VISIBLE"]) == 1
    assert int(v["LKAS_REQUEST"]) == 0
    assert int(v["CTR"]) == 4
    assert int(v["CHKSUM"]) == _checksum(4, 0, 1, 0, 0, 1)

  def test_err_bit_zeros_request_and_copies_err(self):
    v = _decode_cam_lkas(_pack(self.CP, 6, 114, _lkas(lnv=1, err1=1, err2=1))[1])
    assert int(v["LINE_NOT_VISIBLE"]) == 1
    assert int(v["ERR_BIT_1"]) == 1
    assert int(v["ERR_BIT_2"]) == 1
    assert int(v["LKAS_REQUEST"]) == 0

  def test_lnv_1_to_0_resumes_request(self):
    blocked = _decode_cam_lkas(_pack(self.CP, 1, 50, _lkas(lnv=1))[1])
    allowed = _decode_cam_lkas(_pack(self.CP, 2, 50, _lkas(lnv=0))[1])
    assert int(blocked["LKAS_REQUEST"]) == 0
    assert int(blocked["LINE_NOT_VISIBLE"]) == 1
    assert int(allowed["LKAS_REQUEST"]) == 50
    assert int(allowed["LINE_NOT_VISIBLE"]) == 0

  def test_lnv_0_to_1_stops_request(self):
    allowed = _decode_cam_lkas(_pack(self.CP, 1, 50, _lkas(lnv=0))[1])
    blocked = _decode_cam_lkas(_pack(self.CP, 2, 50, _lkas(lnv=1))[1])
    assert int(allowed["LKAS_REQUEST"]) == 50
    assert int(blocked["LKAS_REQUEST"]) == 0
    assert int(blocked["LINE_NOT_VISIBLE"]) == 1

  def test_counter_still_frame_mod_16(self):
    for frame in range(40):
      v = _decode_cam_lkas(_pack(self.CP, frame, 0, _lkas(lnv=0))[1])
      assert int(v["CTR"]) == frame % 16

  def test_checksum_tracks_lnv_and_torque(self):
    for lnv, torque, expect_torque in ((0, 12, 12), (1, 12, 0), (0, 0, 0)):
      v = _decode_cam_lkas(_pack(self.CP, 9, torque, _lkas(lnv=lnv))[1])
      assert int(v["LKAS_REQUEST"]) == expect_torque
      assert int(v["CHKSUM"]) == _checksum(9, expect_torque, lnv, 0, 0, 1)


class TestRoutePrefaultFrames:
  def setup_method(self):
    self.CP, _ = _cp()
    self.cp = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)

  def _fsc(self, dat):
    self.cp.update([(0, [(0x243, dat, 0)])])
    return dict(self.cp.vl["CAM_LKAS"])

  def test_route36_prefault_fsc_lnv1_old_mismatch_removed(self):
    fsc = self._fsc(ROUTE36_FSC_PREFAULT)
    assert int(fsc["LINE_NOT_VISIBLE"]) == 1
    assert int(fsc["LKAS_REQUEST"]) == 0
    assert int(fsc["ERR_BIT_1"]) == 0
    packed = _decode_cam_lkas(_pack(self.CP, 13, 90, fsc)[1])
    assert int(packed["LINE_NOT_VISIBLE"]) == 1
    assert int(packed["LKAS_REQUEST"]) == 0
    assert int(packed["ERR_BIT_1"]) == 0

  def test_route36_fault_frame_does_not_request_steer(self):
    fsc = self._fsc(ROUTE36_FSC_FAULT)
    assert int(fsc["ERR_BIT_1"]) == 1
    packed = _decode_cam_lkas(_pack(self.CP, 2, 118, fsc)[1])
    assert int(packed["LINE_NOT_VISIBLE"]) == 1
    assert int(packed["ERR_BIT_1"]) == 1
    assert int(packed["LKAS_REQUEST"]) == 0


class TestCarControllerLnvGate:
  def setup_method(self):
    self.CP, self.CP_SP = _cp()
    self.CI = CarInterface(self.CP, self.CP_SP)
    self.packer = CANPacker("mazda_2017")
    self.t = 0

  def _step_fsc(self, *, lnv, err1=0, torque=0.2):
    self.t += 10_000_000
    packed = mazdacan.create_steering_control(self.packer, self.CP, 0, 0,
                                              _lkas(lnv=lnv, err1=err1, err2=err1))
    cam = (packed[0], packed[1], 2)
    self.CI.update([(self.t, [cam])])
    CC = structs.CarControl()
    CC.latActive = True
    CC.actuators.torque = torque
    CC_SP = structs.CarControlSP()
    actuators, sends = self.CI.apply(CC.as_reader(), CC_SP, self.t)
    lkas = next(s for s in sends if s[0] == 0x243)
    return actuators, _decode_cam_lkas(lkas[1])

  def test_controller_lnv0_can_request_steer(self):
    actuators, v = self._step_fsc(lnv=0)
    assert int(v["LINE_NOT_VISIBLE"]) == 0
    assert abs(int(v["LKAS_REQUEST"])) > 0
    assert abs(actuators.torqueOutputCan) > 0

  def test_controller_lnv1_keeps_mads_path_but_zeros_wire(self):
    actuators, v = self._step_fsc(lnv=1)
    assert int(v["LINE_NOT_VISIBLE"]) == 1
    assert int(v["LKAS_REQUEST"]) == 0
    assert actuators.torqueOutputCan == 0

  def test_controller_err_bit_zeros_wire(self):
    _, v = self._step_fsc(lnv=0, err1=1)
    assert int(v["ERR_BIT_1"]) == 1
    assert int(v["LKAS_REQUEST"]) == 0
