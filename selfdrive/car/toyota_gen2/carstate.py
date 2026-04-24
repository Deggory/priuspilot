# PriusPilot — Prius Gen 2 (NHW20) car state parser
# Primary CAN parser  → ocelot_controls.dbc (ocelot hardware on bus 0)
# Body/native parser  → prius_gen2_pt.dbc   (native Prius signals on bus 0)
from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import mean
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.toyota_gen2.values import DBC, GEAR_MAP, DetectedEcus

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.cruise_active = False
    self.armed = False
    self.enabled = False
    self.enabled_last = True
    self.setSpeed = 0.0
    self.prev_btn_states = {
      "ON_OFF": False,
      "RES_UP": False,
      "SET_DOWN": False,
      "CANCEL": False,
    }

  def update(self, cp, cp_body):
    """
    cp      — ocelot_controls.dbc parser (ocelot hardware messages)
    cp_body — prius_gen2_pt.dbc parser   (native Prius vehicle messages)
    """
    ret = car.CarState.new_message()

    # ── Steering ──────────────────────────────────────────────────────────────
    # Prefer ocelot SAS (0x50) when SteerInterceptor or SteerActuatorSSC is
    # present; fall back to native Prius SAS (0x25) via cp_body.
    if DetectedEcus["SteerInterceptor"]:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
      ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
      ret.steeringTorque = (
        (cp.vl["INTERCEPTOR_STEERING_SENSOR"]["TRQ_2"] -
         cp.vl["INTERCEPTOR_STEERING_SENSOR"]["TRQ_1"]) / 2
      ) + 100
      ret.steeringTorqueEps = ret.steeringTorque * 100
      ret.steeringPressed = cp.vl["INTERCEPTOR_STEERING_SENSOR"]["OVERRIDE"] != 0
      ret.steerFaultTemporary = cp.vl["INTERCEPTOR_STEERING_SENSOR"]["STATE"] != 0
    elif DetectedEcus["SteerActuator"]:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
      ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
      ret.steeringTorque = cp.vl["ACTUATOR_STEERING_STATUS"]["STEERING_TORQUE_DRIVER"]
      ret.steeringTorqueEps = cp.vl["ACTUATOR_STEERING_STATUS"]["STEERING_TORQUE_EPS"]
      ret.steeringPressed = abs(ret.steeringTorque) > 1000
      ret.steerFaultTemporary = cp.vl["ACTUATOR_STEERING_STATUS"]["STEERING_OK"] != 0
    elif DetectedEcus["SteerActuatorSSC"]:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
      ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
      ret.steeringTorqueEps = cp.vl["STEERING_STATUS_SSC"]["STEERING_TORQUE"]
      ret.steeringTorque = 0.0
      ret.steeringPressed = False
      ret.steerFaultTemporary = False
    else:
      # Phase 1 fallback: native Prius SAS
      ret.steeringAngleDeg = (
        cp_body.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] +
        cp_body.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
      )
      ret.steeringRateDeg = cp_body.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
      ret.steeringTorque = 0.0
      ret.steeringTorqueEps = 0.0
      ret.steeringPressed = False
      ret.steerFaultTemporary = False

    ret.steerFaultPermanent = False

    # ── Speed ─────────────────────────────────────────────────────────────────
    # Use ocelot SPEED (0x76) when Pyboard is running, else native Prius 0x3CA.
    if cp.can_valid:
      vehicle_speed_kph = cp.vl["SPEED"]["CAN_SPEED"] * CV.KPH_TO_MS
    else:
      vehicle_speed_kph = cp_body.vl["SPEED"]["SPEED"] * CV.KPH_TO_MS

    ret.wheelSpeeds = self.get_wheel_speeds(
      vehicle_speed_kph, vehicle_speed_kph,
      vehicle_speed_kph, vehicle_speed_kph,
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr,
                        ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.001

    # ── Gear (native Prius) ────────────────────────────────────────────────────
    can_gear = int(cp_body.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(GEAR_MAP.get(can_gear))

    # ── Brake ─────────────────────────────────────────────────────────────────
    if DetectedEcus["iBooster"]:
      ret.brakePressed = bool(cp.vl["IBOOSTER_BRAKE_STATUS"]["DRIVER_BRAKE_APPLIED"])
    elif DetectedEcus["BrakeActuator"]:
      ret.brakePressed = bool(cp.vl["ACTUATOR_BRAKE_STATUS"]["DRIVER_BRAKE_APPLIED"])
    else:
      # Native Prius brake pedal position (from prius_gen2_pt.dbc)
      ret.brakePressed = cp_body.vl["BRAKE_MODULE"]["BRAKE_POSITION"] > 5
    ret.brakeHoldActive = False

    # ── Gas ───────────────────────────────────────────────────────────────────
    if DetectedEcus["GasInterceptor"]:
      ret.gas = (
        cp.vl["PEDAL_GAS_SENSOR"]["PED_GAS"] +
        cp.vl["PEDAL_GAS_SENSOR"]["PED_GAS2"]
      ) / 2.0
      ret.gasPressed = ret.gas > 15
    elif DetectedEcus["GasActuator"]:
      ret.gas = cp.vl["ACTUATOR_GAS_SENSOR"]["THROTTLE_POS"]
      ret.gasPressed = False
    else:
      ret.gas = cp_body.vl["GAS_PEDAL_HYBRID"]["GAS_PEDAL"]
      ret.gasPressed = ret.gas > 5

    # ── Doors / seatbelt (native Prius) ───────────────────────────────────────
    door_bits = cp_body.vl["DOORS_STATUS"]["DOOR_OPEN"]
    ret.doorOpen = door_bits != 0
    ret.seatbeltUnlatched = False
    ret.parkingBrake = (can_gear == 0)

    # ── Blinkers (RelayCore if present, else unknown) ─────────────────────────
    if DetectedEcus["RelayCore"]:
      relay = int(cp.vl["RELAY_CORE_STATUS"]["RELAY_STATUS"])
      ret.leftBlinker = bool(relay & 0b10)
      ret.rightBlinker = bool(relay & 0b01)
    else:
      ret.leftBlinker = False
      ret.rightBlinker = False

    # ── Cruise state (ocelot software cruise) ─────────────────────────────────
    # The Pyboard sends a CRUISE message (0x100) with button states.
    # We implement a simple armed/enabled state machine here.
    if cp.can_valid:
      on_off = bool(cp.vl["CRUISE"]["ON_OFF"])
      res_up = bool(cp.vl["CRUISE"]["RES_UP"])
      set_dn = bool(cp.vl["CRUISE"]["SET_DOWN"])
      cancel = bool(cp.vl["CRUISE"]["CANCEL"])

      if on_off and not self.prev_btn_states["ON_OFF"]:
        self.armed = not self.armed
        if not self.armed:
          self.enabled = False

      if self.armed:
        if self.enabled:
          self.enabled_last = True
          if res_up and not self.prev_btn_states["RES_UP"]:
            self.setSpeed += 1 * CV.MPH_TO_MS
          if set_dn and not self.prev_btn_states["SET_DOWN"]:
            if self.setSpeed >= 10 * CV.MPH_TO_MS:
              self.setSpeed -= 1 * CV.MPH_TO_MS
          if cancel and not self.prev_btn_states["CANCEL"]:
            self.enabled = False
        else:
          if set_dn and not self.prev_btn_states["SET_DOWN"]:
            self.setSpeed = ret.vEgo
            self.enabled = True
          if res_up and not self.prev_btn_states["RES_UP"] and self.enabled_last:
            self.enabled = True

      self.prev_btn_states["ON_OFF"] = on_off
      self.prev_btn_states["RES_UP"] = res_up
      self.prev_btn_states["SET_DOWN"] = set_dn
      self.prev_btn_states["CANCEL"] = cancel
    else:
      # Fall back to native Prius cruise (Phase 1)
      self.cruise_active = bool(cp_body.vl["CRUISE_CONTROL"]["CRUISE_ACTIVE"])
      self.armed = self.cruise_active
      self.enabled = self.cruise_active

    if ret.brakePressed and self.enabled:
      self.enabled = False

    ret.cruiseState.available = self.armed
    ret.cruiseState.enabled = self.enabled
    ret.cruiseState.speed = self.setSpeed
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = not self.armed

    # ── Misc ──────────────────────────────────────────────────────────────────
    ret.espDisabled = False
    ret.genericToggle = False
    ret.stockAeb = False

    return ret

  @staticmethod
  def get_can_parser(CP):
    """Primary parser: ocelot_controls.dbc on bus 0."""
    signals = [
      # Ocelot SAS (0x50) — present when any steer ECU detected
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),
      ("STEER_RATE", "STEER_ANGLE_SENSOR"),
      # Ocelot speed (0x76) from Pyboard
      ("CAN_SPEED", "SPEED"),
      ("VSS_PULSE_US", "SPEED"),
      # Ocelot cruise buttons (0x100)
      ("ON_OFF", "CRUISE"),
      ("RES_UP", "CRUISE"),
      ("SET_DOWN", "CRUISE"),
      ("CANCEL", "CRUISE"),
      ("MODE", "CRUISE"),
      # Ocelot engine (0x75)
      ("IGNITION_ON", "ENGINE"),
    ]
    checks = [
      ("STEER_ANGLE_SENSOR", 20),
      ("SPEED", 20),
      ("CRUISE", 20),
      ("ENGINE", 20),
    ]

    if DetectedEcus["GasInterceptor"]:
      signals += [
        ("PED_GAS", "PEDAL_GAS_SENSOR"),
        ("PED_GAS2", "PEDAL_GAS_SENSOR"),
        ("STATE", "PEDAL_GAS_SENSOR"),
      ]
      checks += [("PEDAL_GAS_SENSOR", 20)]

    if DetectedEcus["GasActuator"]:
      signals += [("THROTTLE_POS", "ACTUATOR_GAS_SENSOR")]
      checks += [("ACTUATOR_GAS_SENSOR", 20)]

    if DetectedEcus["SteerInterceptor"]:
      signals += [
        ("TRQ_1", "INTERCEPTOR_STEERING_SENSOR"),
        ("TRQ_2", "INTERCEPTOR_STEERING_SENSOR"),
        ("STATE", "INTERCEPTOR_STEERING_SENSOR"),
        ("OVERRIDE", "INTERCEPTOR_STEERING_SENSOR"),
      ]
      checks += [("INTERCEPTOR_STEERING_SENSOR", 20)]

    if DetectedEcus["SteerActuator"]:
      signals += [
        ("STEERING_TORQUE_EPS", "ACTUATOR_STEERING_STATUS"),
        ("STEERING_TORQUE_DRIVER", "ACTUATOR_STEERING_STATUS"),
        ("STEERING_OK", "ACTUATOR_STEERING_STATUS"),
        ("STATUS", "ACTUATOR_STEERING_STATUS"),
      ]
      checks += [("ACTUATOR_STEERING_STATUS", 20)]

    if DetectedEcus["SteerActuatorSSC"]:
      signals += [
        ("STEERING_ANGLE", "STEERING_STATUS_SSC"),
        ("STEERING_SPEED", "STEERING_STATUS_SSC"),
        ("STEERING_TORQUE", "STEERING_STATUS_SSC"),
        ("CONTROL_STATUS", "STEERING_STATUS_SSC"),
      ]
      checks += [("STEERING_STATUS_SSC", 20)]

    if DetectedEcus["iBooster"]:
      signals += [
        ("DRIVER_BRAKE_APPLIED", "IBOOSTER_BRAKE_STATUS"),
        ("BRAKE_OK", "IBOOSTER_BRAKE_STATUS"),
        ("STATUS", "IBOOSTER_BRAKE_STATUS"),
      ]
      checks += [("IBOOSTER_BRAKE_STATUS", 20)]

    if DetectedEcus["BrakeActuator"]:
      signals += [
        ("DRIVER_BRAKE_APPLIED", "ACTUATOR_BRAKE_STATUS"),
        ("BRAKE_OK", "ACTUATOR_BRAKE_STATUS"),
        ("STATUS", "ACTUATOR_BRAKE_STATUS"),
      ]
      checks += [("ACTUATOR_BRAKE_STATUS", 20)]

    if DetectedEcus["RelayCore"]:
      signals += [("RELAY_STATUS", "RELAY_CORE_STATUS")]
      checks += [("RELAY_CORE_STATUS", 20)]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    """Body/native parser: prius_gen2_pt.dbc on bus 0 for native Prius signals."""
    signals = [
      # Native Prius SAS (0x25) — fallback steering
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR"),
      ("STEER_RATE", "STEER_ANGLE_SENSOR"),
      # Brake pedal (0x30)
      ("BRAKE_POSITION", "BRAKE_MODULE"),
      # Gas pedal (0x244)
      ("GAS_PEDAL", "GAS_PEDAL_HYBRID"),
      # Vehicle speed (0x3CA)
      ("SPEED", "SPEED"),
      # Gear selector (0x120)
      ("GEAR", "GEAR_PACKET"),
      # Doors (0x5B6)
      ("DOOR_OPEN", "DOORS_STATUS"),
      # Native cruise (0x5C8) — used when ocelot CRUISE not available
      ("CRUISE_ACTIVE", "CRUISE_CONTROL"),
    ]
    checks = [
      ("STEER_ANGLE_SENSOR", 77),
      ("SPEED", 9),
      ("GEAR_PACKET", 59),
      ("BRAKE_MODULE", 40),
      ("GAS_PEDAL_HYBRID", 40),
      ("DOORS_STATUS", 0),
      ("CRUISE_CONTROL", 0),
    ]
    return CANParser(DBC[CP.carFingerprint]["body"], signals, checks, 0)
