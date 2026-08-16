from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import Buttons, MazdaFlags

# Route 3C OEM FSC bus2 (ff7df7d6f9c3403b|0000003c--c56c36c84d): LNV=1 + nonzero
# LKAS_REQUEST max=177, p99=133. Route 39 faulted at LNV=1 + 1088. This is a
# magnitude envelope, not a permission gate — LNV=1 still allows steering.
FSC_LNV1_LKAS_REQUEST_MAX = 200

# Radar frames the body ECU expects to keep receiving for stop-and-go to work. Byte-exact
# captures from a 0x764 radar with no objects in view; only the counter nibble in the last
# byte changes. 0x364 optionally carries a synthetic stopped lead so standstill holds work
# without a real lead in radar view.
RADAR_STATIC_MSG = (0x499, bytes.fromhex("0008c00000000000"))
RADAR_TRACK_MSGS = {
  0x361: bytes.fromhex("fff7fefe1fc00080"),
  0x362: bytes.fromhex("fff7fefe1fc78c80"),
  0x363: bytes.fromhex("fff7fefe1fc00000"),
  0x364: bytes.fromhex("fff7fefe1fc00000"),
  0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
  0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
}
SYNTHETIC_LEAD_TRACK_ADDR = 0x364
SYNTHETIC_LEAD_TRACK_MSG = bytes.fromhex("0a4000001dc00000")


def crz_info_checksum(dat: bytes) -> int:
  # Inverted sum of the first seven bytes; the radar leaves the STOPPING bit out of the
  # sum. Verified against 1.94M stock frames, including every stop-bit frame.
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04)) & 0xFF)) & 0xFF


def create_acc_command(packer, bus, counter, accel, long_active, acc_available, stopping, resume_unlatching):
  # CRZ_INFO stands in for the disabled radar's accel command frame. While MRCC is armed
  # but not engaged, stock advertises ACC_SET_ALLOWED with a zero command so the dash
  # accepts SET; with the main switch off it broadcasts a static standby pattern with the
  # command field pegged high.
  values = {
    "STATUS": 1,
    "STATIC_1": 0x7ff,
    "CTR1": counter % 16,
  }
  if long_active or acc_available:
    values.update({
      "ACCEL_CMD": accel,
      "ACC_ACTIVE": int(long_active),
      "ACC_SET_ALLOWED": 1,
      "NEW_SIGNAL_7": 1,
      "STOPPING": int(stopping),
      "STOPPING_2": int(stopping),
      "RESUME_UNLATCHING": int(resume_unlatching),
    })
  else:
    values["ACCEL_CMD"] = 4.094  # standby pattern, raw 8190

  dat = packer.make_can_msg("CRZ_INFO", bus, values)[1]
  values["CHKSUM"] = crz_info_checksum(dat)
  return packer.make_can_msg("CRZ_INFO", bus, values)


def create_crz_ctrl(packer, bus, long_active, acc_available, gap_setting, radar_has_lead, stop_go_phase, acc_active_2):
  # CRZ_CTRL stands in for the disabled radar's cruise-state frame. stop_go_phase mirrors
  # stock's stop-and-go progression through RADAR_LEAD_RELATIVE_DISTANCE (see the DBC
  # comment); gap_setting mirrors the driver's distance setting on the dash.
  values = {
    "MSG_1_INV": 1,
    "MSG_1_INV_COPY": 1,
    "NEW_SIGNAL_8": 1,
    "CRZ_ACTIVE": int(long_active),
    "CRZ_AVAILABLE": int(long_active or acc_available),
    "DISTANCE_SETTING": gap_setting,
    "RADAR_HAS_LEAD": int(radar_has_lead),
    "RADAR_LEAD_RELATIVE_DISTANCE": stop_go_phase,
    "ACC_ACTIVE_2": int(acc_active_2),
  }
  return packer.make_can_msg("CRZ_CTRL", bus, values)


def create_radar_frames(bus, counter, synthetic_lead):
  frames = [CanData(RADAR_STATIC_MSG[0], RADAR_STATIC_MSG[1], bus)]
  for addr, dat in RADAR_TRACK_MSGS.items():
    if synthetic_lead and addr == SYNTHETIC_LEAD_TRACK_ADDR:
      dat = SYNTHETIC_LEAD_TRACK_MSG
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xf0) | (counter % 16)]), bus))
  return frames


def fsc_cam_lkas_allows_steer(lkas) -> bool:
  # FSC ERR_BIT_1/2 is a real camera fault (steerFaultPermanent). Do not keep
  # requesting torque against that. LINE_NOT_VISIBLE is a CAM_LKAS field to copy
  # for coherence, not a steering-permission gate: public FSC bus2
  # 60ed2cf2b490292c|2023-11-21--17-47-15 sent LNV=1 with nonzero LKAS_REQUEST
  # (133 checksum-valid frames, 4 runs) and the CX-5 2025 faulting routes
  # mismatched comma LNV=0 vs FSC LNV=1, not "nonzero request while LNV=1".
  return int(lkas.get("ERR_BIT_1", 0)) == 0 and int(lkas.get("ERR_BIT_2", 0)) == 0


def clip_lkas_request_to_fsc_envelope(apply_torque: int, lkas) -> int:
  # Do not zero LNV=1. Cap only the magnitude to the OEM LNV=1 envelope.
  if int(lkas.get("LINE_NOT_VISIBLE", 1)) == 1 and abs(apply_torque) > FSC_LNV1_LKAS_REQUEST_MAX:
    return FSC_LNV1_LKAS_REQUEST_MAX if apply_torque > 0 else -FSC_LNV1_LKAS_REQUEST_MAX
  return int(apply_torque)


def hud_tja_for_request(cam_laneinfo: dict, cam_lkas: dict | None, apply_torque: int) -> int:
  # Route 3C OEM: TJA=0 never has nonzero CAM_LKAS (340/340 request=0).
  # TJA=4 is the sustained-active HUD/state with LNV=0 (99.5% nonzero).
  # TJA=3 is the LNV=1 drop/transition with brief nonzero request.
  # Parked isolated TJA=4 without a matching CAM_LKAS request faulted — never
  # send TJA=4 unless we are actually requesting with LNV=0.
  tja = int(cam_laneinfo.get("TJA", 0))
  if abs(apply_torque) < 1:
    return tja
  lnv = int((cam_lkas or {}).get("LINE_NOT_VISIBLE", cam_laneinfo.get("LINE_NOT_VISIBLE", 1)))
  if lnv == 0:
    return 4
  return 3


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = int(lkas.get("LINE_NOT_VISIBLE", 1))
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  if not fsc_cam_lkas_allows_steer(lkas):
    apply_torque = 0
  apply_torque = clip_lkas_request_to_fsc_envelope(apply_torque, lkas)

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool,
                         apply_torque: int = 0, cam_lkas: dict | None = None):
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # Copy FSC TJA. Packer default 0 is OEM OFF; Route 39 TX'd TJA=0 while
    # commanding CAM_LKAS, a combination that never occurs on golden OEM 3C.
    "TJA": hud_tja_for_request(cam_msg, cam_lkas, apply_torque),
    "TJA_TRANSITION": int(cam_msg.get("TJA_TRANSITION", 0)),
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": inc,
      "SET_P_INV": (inc + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": dec,
      "SET_M_INV": (dec + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
