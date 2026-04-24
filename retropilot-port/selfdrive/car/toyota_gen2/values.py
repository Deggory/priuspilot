# Prius Gen 2 (NHW20, 2004-2009) car definitions for retropilot (openpilot)
from selfdrive.car import dbc_dict


class CAR:
  PRIUS_GEN2 = "TOYOTA PRIUS GEN2 2004"


# DBC file mapping — no ADAS bus on Gen 2
DBC = {
  CAR.PRIUS_GEN2: dbc_dict('prius_gen2_pt', None),
}

# Gear values (CAN message 0x0120, bits 0-1 of byte 3)
GEAR_MAP = {
  0: "P",
  1: "R",
  2: "N",
  3: "D",
}

# CAN fingerprint — (address: dlc) pairs seen on bus 0 in READY mode
FINGERPRINTS = {
  CAR.PRIUS_GEN2: [{
    0x0022: 8,   # Lateral acceleration (~77 Hz)
    0x0023: 8,   # Longitudinal acceleration (~77 Hz)
    0x0025: 8,   # Steering angle sensor (~77 Hz)
    0x0030: 8,   # Brake pedal position (~167 Hz)
    0x0038: 8,   # ICE RPM coarse (~25 Hz)
    0x0039: 8,   # ICE coolant temperature (~11 Hz)
    0x003B: 8,   # Electric motor current / HV voltage (~125 Hz)
    0x0081: 8,   # Front wheel pulses (~77 Hz)
    0x0083: 8,   # Rear wheel pulses (~77 Hz)
    0x0120: 8,   # Gear selector / drive mode (~59 Hz)
    0x0244: 8,   # Throttle pedal position (~40 Hz)
    0x03CA: 8,   # Vehicle speed (~9 Hz)
    0x03CB: 8,   # HV battery state (~9 Hz)
    0x0529: 8,   # Event messages (~1 Hz)
    0x05B6: 8,   # Doors / hatch status (~1 Hz)
    0x05CC: 8,   # Outside temperature (~3 Hz)
  }],
}
