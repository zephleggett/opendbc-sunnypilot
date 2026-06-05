"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
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

# If the carcontroller skipped us for this many frames or more, the previous
# cycle was suppressed (cancel/resume gate in carcontroller). Reset the
# long-press state machine to idle so we don't resume a stale mid-hold from
# the wrong starting point. The threshold is a >= so a 5-frame (50 ms) gap
# also triggers the reset (off-by-one was caught in review).
SUPPRESSION_GAP_FRAMES = 5

# Minimum hold frames committed once a long-press starts. Without a commit,
# small jitter around the LONG_PRESS_THRESHOLD_MPH boundary (~5 mph remaining
# drop) would cause the SM to thrash between long and short paths, restarting
# the body-ECU recognition window each time. Once a hold is started we stay
# in the long-press SM until at least this many hold frames have elapsed.
LONG_PRESS_MIN_COMMIT_FRAMES = 20  # 0.2 s -- well below the body ECU's ~0.5 s recognition window


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    # Long-press state machine
    self.lp_phase = 'idle'  # 'idle' | 'holding' | 'cooldown'
    self.lp_phase_frames = 0
    self.lp_last_send_button = SendButtonState.none
    # Per-send CTR ratchet, so 30 sustained sends across a 0.6 s hold do not
    # collide on the wire (the wheel's crz_btns_counter only advances every
    # 100 ms at 10 Hz, so without our own ratchet the body ECU could see the
    # same CTR repeatedly and ignore duplicates).
    self.lp_ctr_offset = 1
    # Last frame in which update() was called. Used to detect suppression
    # gaps from the carcontroller's cancel/resume guard so we don't resume
    # a stale long-press from the middle.
    self.lp_last_called_frame = -1

  def _reset_long_press_state(self, send_button=SendButtonState.none):
    self.lp_phase = 'idle'
    self.lp_phase_frames = 0
    self.lp_last_send_button = send_button
    self.lp_ctr_offset = 1

  def update(self, CC_SP, CS, packer, frame, last_button_frame) -> list[CanData]:
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    # If we were suppressed for at least SUPPRESSION_GAP_FRAMES cycles
    # (cancel/resume/driver cancel button held), the long-press state machine
    # could be mid-hold with frames that no longer correspond to a continuous
    # hold on the wire. Reset. (Using >= so the boundary case is covered.)
    if self.lp_last_called_frame >= 0 and (frame - self.lp_last_called_frame) >= SUPPRESSION_GAP_FRAMES:
      self._reset_long_press_state()
    self.lp_last_called_frame = frame

    if self.ICBM.sendButton == SendButtonState.none:
      self._reset_long_press_state()
      return []

    send_button = BUTTONS[self.ICBM.sendButton]

    # If the controller flipped direction mid-cycle, reset the long-press SM
    # so we don't accidentally hold the opposite button.
    if self.ICBM.sendButton != self.lp_last_send_button:
      self._reset_long_press_state(self.ICBM.sendButton)

    # Remaining magnitude (mph) between current dash and target.
    # ICBM.vTarget is the rounded integer in the locale's speed unit (mph
    # for imperial Mazda US). CS.cruiseState.speed is m/s.
    # TODO-SP: handle metric Mazda variants (10 kph long-press step there).
    target_mph = int(self.ICBM.vTarget)
    dash_mph = int(round(CS.cruiseState.speed * CV.MS_TO_MPH))
    remaining_drop_mph = abs(dash_mph - target_mph)

    # Mazda's body ECU clamps CRZ_SPEED at MRCC's 19 mph floor on its own --
    # commanding a 5 mph step from dash<24 just bottoms out at 19 instead of
    # going below, so no explicit floor guard is needed here.

    # Mode select with a one-cycle commit to prevent threshold thrash. Once a
    # hold has started, stay in the long-press path for at least
    # LONG_PRESS_MIN_COMMIT_FRAMES so the body ECU's recognition window is
    # not restarted by transient remaining_drop dips just below the threshold.
    committed_to_long = (self.lp_phase != 'idle'
                         and self.lp_phase_frames < LONG_PRESS_MIN_COMMIT_FRAMES)
    if remaining_drop_mph >= LONG_PRESS_THRESHOLD_MPH or committed_to_long:
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

    Counter strategy: each transmitted frame gets a strictly-increasing CTR
    offset (mod 16) on top of the wheel's current crz_btns_counter, so the
    body ECU never sees two of our consecutive sends sharing a CTR.
    """
    can_sends = []

    if self.lp_phase == 'idle':
      # lp_ctr_offset is already reset to 1 by _reset_long_press_state on
      # every entry to idle (sendButton=none, direction flip, or suppression
      # gap), so no separate init is needed here.
      self.lp_phase = 'holding'
      self.lp_phase_frames = 0

    if self.lp_phase == 'holding':
      if self.lp_phase_frames < LONG_PRESS_HOLD_FRAMES:
        if self.lp_phase_frames % LONG_PRESS_SEND_EVERY == 0:
          ctr_arg = (CS.crz_btns_counter + self.lp_ctr_offset) % 16
          can_sends.append(mazdacan.create_button_cmd(
            packer, self.CP, ctr_arg, send_button
          ))
          self.lp_ctr_offset = (self.lp_ctr_offset + 1) % 16
          if self.lp_ctr_offset == 0:
            self.lp_ctr_offset = 1  # avoid wheel's exact CTR
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
    # Reset the long-press SM via the shared helper so lp_ctr_offset and
    # lp_last_send_button stay coherent. We preserve the current direction
    # so a long->short transition is not treated as a direction flip.
    self._reset_long_press_state(self.ICBM.sendButton)

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
