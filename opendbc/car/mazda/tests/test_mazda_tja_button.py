#!/usr/bin/env python3
"""TJA-only MADS button contract for Mazda CX-5 2022+ openpilot long.

Physical TJA rising edge emits ButtonType.lkas once. MRCC is OEM-only and must
not emit lkas or mainCruise. cruiseState follows PEDALS, never a TJA override.
Stock longitudinal leaves TJA out of the openpilot-long MADS path.
"""

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

ButtonType = structs.CarState.ButtonEvent.Type


def _ci(*, alpha_long=True):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  assert CP.openpilotLongitudinalControl == alpha_long
  return CarInterface(CP, CP_SP)


def _crz_btns(packer, *, tja=0, mrcc=0, set_p=0, set_m=0, res=0, can_off=0, mode_xy=0):
  addr, dat, bus = packer.make_can_msg("CRZ_BTNS", 0, {
    "TJA_BUTTON": tja,
    "SET_P": set_p,
    "SET_M": set_m,
    "RES": res,
    "CAN_OFF": can_off,
    "MODE_X": mode_xy,
    "MODE_Y": mode_xy,
    "BIT1": 1,
    "BIT2": 1,
    "BIT3": 1,
  })
  if mrcc:
    # Physical MRCC is DBC bit 15 (byte 1 MSB). Not a production signal.
    dat = bytes((dat[0], dat[1] | 0x80, *dat[2:]))
  return addr, dat, bus


class ButtonHarness:
  def __init__(self, *, alpha_long=True):
    self.ci = _ci(alpha_long=alpha_long)
    self.packer = CANPacker("mazda_2017")
    self.t = 0

  def step(self, *, tja=0, mrcc=0, acc_off=0, acc_active=0, set_p=0, set_m=0,
           res=0, can_off=0, mode_xy=0):
    self.t += 10_000_000
    crz = _crz_btns(self.packer, tja=tja, mrcc=mrcc, set_p=set_p, set_m=set_m,
                    res=res, can_off=can_off, mode_xy=mode_xy)
    pedals = self.packer.make_can_msg("PEDALS", 0, {
      "ACC_OFF": acc_off,
      "ACC_ACTIVE": acc_active,
      "BRAKE_ON": 0,
    })
    cs, _ = self.ci.update([(self.t, [crz, pedals])])
    return cs


def _events(cs, button_type):
  return [be for be in cs.buttonEvents if be.type == button_type]


def _assert_one_lkas_press(cs):
  ev = _events(cs, ButtonType.lkas)
  assert len(ev) == 1
  assert ev[0].pressed
  assert not _events(cs, ButtonType.mainCruise)


class TestMazdaTjaButton:
  def test_tja_rising_edge_emits_one_lkas(self):
    h = ButtonHarness()
    h.step()
    cs = h.step(tja=1)
    _assert_one_lkas_press(cs)

  def test_tja_held_emits_no_additional_lkas(self):
    h = ButtonHarness()
    h.step()
    h.step(tja=1)
    for _ in range(5):
      held = h.step(tja=1)
      assert not _events(held, ButtonType.lkas)
      assert not _events(held, ButtonType.mainCruise)

  def test_tja_release_does_not_press_lkas(self):
    h = ButtonHarness()
    h.step()
    h.step(tja=1)
    release = h.step(tja=0)
    ev = _events(release, ButtonType.lkas)
    assert len(ev) == 1
    assert not ev[0].pressed
    assert not _events(release, ButtonType.mainCruise)

  def test_second_tja_rising_edge_emits_one_lkas(self):
    h = ButtonHarness()
    h.step()
    h.step(tja=1)
    h.step(tja=0)
    cs = h.step(tja=1)
    _assert_one_lkas_press(cs)

  def test_one_lkas_press_per_rising_edge(self):
    h = ButtonHarness()
    h.step()
    presses = 0
    for tja in (1, 0, 1, 0, 1, 0, 1, 0):
      cs = h.step(tja=tja)
      presses += sum(1 for be in _events(cs, ButtonType.lkas) if be.pressed)
    assert presses == 4

  def test_mrcc_does_not_emit_lkas_or_maincruise(self):
    h = ButtonHarness()
    h.step()
    cs = h.step(mrcc=1)
    assert not _events(cs, ButtonType.lkas)
    assert not _events(cs, ButtonType.mainCruise)
    held = h.step(mrcc=1)
    assert not _events(held, ButtonType.lkas)
    assert not _events(held, ButtonType.mainCruise)
    release = h.step(mrcc=0)
    assert not _events(release, ButtonType.lkas)
    assert not _events(release, ButtonType.mainCruise)

  def test_set_res_cancel_do_not_emit_lkas(self):
    h = ButtonHarness()
    h.step()
    for kwargs, expected in (
      ({"set_p": 1}, ButtonType.accelCruise),
      ({"set_m": 1}, ButtonType.decelCruise),
      ({"res": 1}, ButtonType.resumeCruise),
      ({"can_off": 1}, ButtonType.cancel),
    ):
      h.step()
      cs = h.step(**kwargs)
      assert _events(cs, expected)
      assert not _events(cs, ButtonType.lkas)
      assert not _events(cs, ButtonType.mainCruise)

  def test_tja_does_not_fabricate_cruise_available_or_enabled(self):
    h = ButtonHarness()
    off = h.step()
    assert not off.cruiseState.available
    assert not off.cruiseState.enabled
    tja_off = h.step(tja=1)
    assert not tja_off.cruiseState.available
    assert not tja_off.cruiseState.enabled
    h.step(tja=0)

    armed = h.step(acc_off=1)
    assert armed.cruiseState.available
    assert not armed.cruiseState.enabled
    tja_armed = h.step(tja=1, acc_off=1)
    assert tja_armed.cruiseState.available
    assert not tja_armed.cruiseState.enabled
    h.step(acc_off=1)

    active = h.step(acc_off=1, acc_active=1)
    assert active.cruiseState.available
    assert active.cruiseState.enabled
    tja_active = h.step(tja=1, acc_off=1, acc_active=1)
    assert tja_active.cruiseState.available
    assert tja_active.cruiseState.enabled

  def test_mrcc_oem_cruise_parsing_follows_pedals(self):
    h = ButtonHarness()
    h.step()
    # MRCC press with OEM still off: cruise stays off.
    mrcc_off = h.step(mrcc=1)
    assert not mrcc_off.cruiseState.available
    assert not mrcc_off.cruiseState.enabled
    h.step()
    # OEM arms via PEDALS, with or without the MRCC bit.
    armed = h.step(mrcc=1, acc_off=1)
    assert armed.cruiseState.available
    assert not armed.cruiseState.enabled
    assert not _events(armed, ButtonType.lkas)
    h.step(acc_off=1)
    active = h.step(mrcc=1, acc_off=1, acc_active=1)
    assert active.cruiseState.available
    assert active.cruiseState.enabled
    h.step(acc_off=1, acc_active=1)
    off = h.step(mrcc=1, acc_off=0, acc_active=0)
    assert not off.cruiseState.available
    assert not off.cruiseState.enabled
    assert not _events(off, ButtonType.lkas)

  def test_stock_long_does_not_decode_tja(self):
    # Stock long forces tja_button=0, so physical TJA must not emit lkas.
    h = ButtonHarness(alpha_long=False)
    assert not h.ci.CP.openpilotLongitudinalControl
    h.step()
    rising = h.step(tja=1)
    assert not _events(rising, ButtonType.lkas)
    assert not _events(rising, ButtonType.mainCruise)
    held = h.step(tja=1)
    assert not _events(held, ButtonType.lkas)
    release = h.step(tja=0)
    assert not _events(release, ButtonType.lkas)
    second = h.step(tja=1)
    assert not _events(second, ButtonType.lkas)
    assert not _events(second, ButtonType.mainCruise)
