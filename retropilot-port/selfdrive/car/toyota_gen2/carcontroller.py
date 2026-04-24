# Prius Gen 2 (NHW20, 2004-2009) car controller for retropilot (openpilot)
# Phase 1: NO CONTROLS — all actuator commands are no-ops
from opendbc.can.packer import CANPacker
from selfdrive.car.toyota_gen2.values import DBC


class CarController():
  def __init__(self, dbc_name, CP, VM):
    # CP and VM are required by the CarController interface contract but unused in Phase 1
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):
    # Phase 1: send NO CAN messages to the car.
    # Safety model is noOutput so the panda will block anything we send anyway.
    can_sends = []

    new_actuators = actuators.copy()
    return new_actuators, can_sends
