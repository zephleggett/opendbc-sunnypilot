"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import Buttons
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: Buttons.SET_PLUS,
  SendButtonState.decrease: Buttons.SET_MINUS,
}

# Mazda body ECU's hardcoded long-press behavior:
#   - Sustained CRZ_BTNS hold past ~0.5 s triggers exactly a 5 mph step in
#     CRZ_SPEED. Continuing to hold triggers additional 5-mph cycles ~0.5 s
#     apart. Empirically verified across 91 wheel-side SET_P holds and 35
#     wheel-side SET_M holds in tools/mazda_long/ rlogs (see
#     /tmp/mazda_hold_rate.py for the survey).
LONG_PRESS_THRESHOLD_MPH = 5
LONG_PRESS_HOLD_FRAMES = 60     # 0.6 s at 100 Hz carcontroller cycle
LONG_PRESS_COOLDOWN_FRAMES = 30  # 0.3 s release between cycles
LONG_PRESS_SEND_EVERY = 2        # 50 Hz send rate during a hold (overpowers wheel's 10 Hz)


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    # Long-press state machine
    self.lp_phase = 'idle'  # 'idle' | 'holding' | 'cooldown'
    self.lp_phase_frames = 0
    self.lp_last_send_button = SendButtonState.none

  def update(self, CC_SP, CS, packer, frame, last_button_frame) -> list[CanData]:
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton == SendButtonState.none:
      self.lp_phase = 'idle'
      self.lp_phase_frames = 0
      self.lp_last_send_button = SendButtonState.none
      return []

    send_button = BUTTONS[self.ICBM.sendButton]

    # If the controller flipped direction mid-cycle, reset the long-press SM
    # so we don't accidentally hold the opposite button.
    if self.ICBM.sendButton != self.lp_last_send_button:
      self.lp_phase = 'idle'
      self.lp_phase_frames = 0
      self.lp_last_send_button = self.ICBM.sendButton

    # Remaining magnitude (mph) between current dash and target.
    # ICBM.vTarget is the rounded integer in the locale's speed unit (mph
    # for imperial Mazda US). CS.cruiseState.speed is m/s.
    # TODO-SP: handle metric Mazda variants (10 kph long-press step there).
    target_mph = int(self.ICBM.vTarget)
    dash_mph = int(round(CS.cruiseState.speed * 2.237))
    remaining_drop_mph = abs(dash_mph - target_mph)

    if remaining_drop_mph >= LONG_PRESS_THRESHOLD_MPH:
      return self._send_long_press(send_button, packer, CS)
    return self._send_short_press(send_button, packer, CS)

  def _send_long_press(self, send_button, packer, CS) -> list[CanData]:
    """Sustained CRZ_BTNS hold to trigger Mazda's hardcoded 5 mph step.

    State machine:
      idle    -> holding (on entry)
      holding -> cooldown (after LONG_PRESS_HOLD_FRAMES frames)
      cooldown -> idle    (after LONG_PRESS_COOLDOWN_FRAMES frames)

    During holding we transmit at 50 Hz (every other carcontroller cycle).
    This overpowers the wheel's 10 Hz baseline CRZ_BTNS so the body ECU sees
    a sustained press.
    """
    can_sends = []

    if self.lp_phase == 'idle':
      self.lp_phase = 'holding'
      self.lp_phase_frames = 0

    if self.lp_phase == 'holding':
      if self.lp_phase_frames < LONG_PRESS_HOLD_FRAMES:
        if self.lp_phase_frames % LONG_PRESS_SEND_EVERY == 0:
          can_sends.append(mazdacan.create_button_cmd(
            packer, self.CP, (CS.crz_btns_counter + 1) % 16, send_button
          ))
        self.lp_phase_frames += 1
        if self.lp_phase_frames >= LONG_PRESS_HOLD_FRAMES:
          self.lp_phase = 'cooldown'
          self.lp_phase_frames = 0
    elif self.lp_phase == 'cooldown':
      self.lp_phase_frames += 1
      if self.lp_phase_frames >= LONG_PRESS_COOLDOWN_FRAMES:
        self.lp_phase = 'idle'
        self.lp_phase_frames = 0

    return can_sends

  def _send_short_press(self, send_button, packer, CS) -> list[CanData]:
    """Original single-press behavior for sub-5-mph trim adjustments."""
    self.lp_phase = 'idle'
    self.lp_phase_frames = 0

    can_sends = []
    if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
      self.button_frame += 1
      button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
      if button_counter_offset is not None:
        can_sends.append(mazdacan.create_button_cmd(
          packer, self.CP, CS.crz_btns_counter + button_counter_offset, send_button
        ))
        self.last_button_frame = self.frame

    return can_sends
