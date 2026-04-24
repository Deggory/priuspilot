# PriusPilot — Prius Gen 2 (NHW20, 2004-2009) car definitions
from selfdrive.car import dbc_dict


class CAR:
  PRIUS_GEN2 = "TOYOTA PRIUS GEN2 2004"


# DBC file mapping.
# 'pt'   → ocelot_controls.dbc  (ocelot hardware messages: actuator cmds/status,
#           ocelot STEER_ANGLE_SENSOR 0x50, SPEED 0x76, CRUISE 0x100, ENGINE 0x75)
# 'body' → prius_gen2_pt.dbc    (native Prius CAN: gear, brake pos, gas pedal,
#           native STEER_ANGLE_SENSOR 0x25, SPEED 0x3CA, doors, temp, …)
DBC = {
  CAR.PRIUS_GEN2: dbc_dict('ocelot_controls', None, body_dbc='prius_gen2_pt'),
}

# Ocelot ECU presence flags — updated at runtime by interface.py get_params()
# based on which CAN addresses are seen in the live fingerprint.
DetectedEcus = {
  "GasInterceptor":  False,  # 0x201 PEDAL_GAS_SENSOR
  "GasActuator":     False,  # 0x401 ACTUATOR_GAS_SENSOR
  "SteerInterceptor": False, # 0x301 INTERCEPTOR_STEERING_SENSOR
  "SteerActuator":   False,  # 0x12F ACTUATOR_STEERING_STATUS
  "SteerActuatorSSC": False, # 0x22F STEERING_STATUS_SSC
  "iBooster":        False,  # 0x20F IBOOSTER_BRAKE_STATUS
  "BrakeActuator":   False,  # 0x10F ACTUATOR_BRAKE_STATUS
  "RelayCore":       False,  # 0x601 RELAY_CORE_STATUS
}


# Steer torque limits for the ocelot steering interceptor
class SteerLimitParams:
  STEER_MAX = 350
  STEER_DELTA_UP = 5       # torque ramp up (counts per 10 ms frame)
  STEER_DELTA_DOWN = 5     # torque ramp down
  STEER_ERROR_MAX = 350    # max delta between cmd and motor torque


# Gear values mapping (from native Prius CAN message 0x0120)
GEAR_MAP = {
  0: "P",
  1: "R",
  2: "N",
  3: "D",
}

# CAN fingerprint — set of (CAN_address: data_length) pairs seen on bus 0
# when the car is in READY mode. Used for automatic car identification.
FINGERPRINTS = {
  CAR.PRIUS_GEN2: [{
    # Native Prius Gen 2 messages (always present)
    0x0022: 8,   # Lateral acceleration
    0x0023: 8,   # Longitudinal acceleration
    0x0025: 8,   # Steering angle sensor (native Prius SAS)
    0x0030: 8,   # Brake pedal position
    0x0038: 8,   # ICE RPM (coarse)
    0x0039: 8,   # ICE coolant temperature
    0x003B: 8,   # EM current / HV voltage
    0x0081: 8,   # Front wheel pulses
    0x0083: 8,   # Rear wheel pulses
    0x0120: 8,   # Drive mode (gear: P/R/N/D)
    0x0244: 8,   # Throttle pedal position
    0x03CA: 8,   # Vehicle speed (native)
    0x03CB: 8,   # HV battery SOC
    0x0529: 8,   # Event messages
    0x05B6: 8,   # Doors/hatch status
    0x05CC: 8,   # Outside temperature
    # Ocelot Pyboard messages (present when Pyboard is running)
    0x0050: 6,   # Ocelot STEER_ANGLE_SENSOR (0x50)
    0x0075: 8,   # Ocelot ENGINE
    0x0076: 8,   # Ocelot SPEED (CAN_SPEED)
    0x0100: 8,   # Ocelot CRUISE buttons
  }],
}
