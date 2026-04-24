# Prius Gen 2 (NHW20, 2004-2009) car state parser for retropilot (openpilot)
# Phase 1: read-only — no actuator commands
from cereal import car
from common.numpy_fast import mean
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.toyota_gen2.values import DBC, GEAR_MAP

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.cruise_active = False

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # ---- Steering ----
    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + \
                           cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    # Gen 2 has no torque sensor — provide safe defaults
    ret.steeringTorque = 0.0
    ret.steeringTorqueEps = 0.0
    ret.steeringPressed = False
    ret.steerFaultTemporary = False
    ret.steerFaultPermanent = False

    # ---- Speed ----
    # Vehicle speed is broadcast at 0x3CA (~9 Hz) in km/h
    vehicle_speed_kph = cp.vl["SPEED"]["SPEED"]

    # Gen 2 has no per-wheel speed messages — use vehicle speed for all four
    ret.wheelSpeeds = self.get_wheel_speeds(
      vehicle_speed_kph,
      vehicle_speed_kph,
      vehicle_speed_kph,
      vehicle_speed_kph,
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr,
                        ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.001

    # ---- Gear ----
    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    gear_str = GEAR_MAP.get(can_gear, None)
    ret.gearShifter = self.parse_gear_shifter(gear_str)

    # ---- Brake ----
    brake_pos = cp.vl["BRAKE_MODULE"]["BRAKE_POSITION"]
    ret.brakePressed = brake_pos > 5   # threshold to filter noise
    ret.brakeHoldActive = False

    # ---- Gas ----
    ret.gas = cp.vl["GAS_PEDAL_HYBRID"]["GAS_PEDAL"]
    ret.gasPressed = ret.gas > 5       # threshold to filter noise

    # ---- Doors / seatbelt ----
    door_bits = cp.vl["DOORS_STATUS"]["DOOR_OPEN"]
    ret.doorOpen = door_bits != 0
    ret.seatbeltUnlatched = False      # not decoded on Gen 2
    ret.parkingBrake = (can_gear == 0) # P gear == parking brake

    # ---- Blinkers ----
    ret.leftBlinker = False            # not yet decoded on Gen 2
    ret.rightBlinker = False

    # ---- Cruise ----
    self.cruise_active = bool(cp.vl["CRUISE_CONTROL"]["CRUISE_ACTIVE"])
    ret.cruiseState.available = True
    ret.cruiseState.enabled = self.cruise_active
    ret.cruiseState.speed = 0.0        # set speed not available in Phase 1
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = True # Gen 2 has non-adaptive cruise only

    # ---- Misc ----
    ret.espDisabled = False
    ret.genericToggle = False
    ret.stockAeb = False

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # Steering angle sensor (0x025, ~77 Hz)
      ("STEER_ANGLE",    "STEER_ANGLE_SENSOR"),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR"),
      ("STEER_RATE",     "STEER_ANGLE_SENSOR"),

      # Brake pedal position (0x030, ~167 Hz)
      ("BRAKE_POSITION", "BRAKE_MODULE"),

      # Throttle pedal (0x244, ~40 Hz)
      ("GAS_PEDAL",      "GAS_PEDAL_HYBRID"),

      # Vehicle speed (0x3CA, ~9 Hz) — in km/h
      ("SPEED",          "SPEED"),

      # Gear selector (0x120, ~59 Hz)
      ("GEAR",           "GEAR_PACKET"),

      # Doors (0x5B6, ~1 Hz)
      ("DOOR_OPEN",      "DOORS_STATUS"),

      # Cruise control active (0x5C8, ~1 Hz)
      ("CRUISE_ACTIVE",  "CRUISE_CONTROL"),
    ]

    checks = [
      # Critical signals — must arrive at expected rate
      ("STEER_ANGLE_SENSOR", 77),
      ("SPEED",              9),
      ("GEAR_PACKET",        59),
      ("BRAKE_MODULE",       40),   # relaxed from 167 Hz
      ("GAS_PEDAL_HYBRID",   40),

      # Non-critical — freq 0 so missing messages don't invalidate CAN
      ("DOORS_STATUS",       0),
      ("CRUISE_CONTROL",     0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    # Prius Gen 2 has no camera CAN bus
    return None
