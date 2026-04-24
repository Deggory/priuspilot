# Prius Gen 2 Port for retropilot

This directory contains the files needed to add Toyota Prius Gen 2 (NHW20,
2004-2009) car support to the [retropilot](https://github.com/Deggory/retropilot)
openpilot-based fork.

## What is retropilot?

retropilot is an openpilot fork for the comma two (c2) device. It supports both
standard modern Toyotas and a generic "RETROFIT" car type for any vehicle fitted
with Ocelot-based actuator hardware.

## What is this port?

This port adds native Prius Gen 2 CAN bus reading to retropilot's Toyota car
module. It follows the same openpilot car interface conventions used by the
existing retropilot Toyota support, adapted for the NHW20's pre-ADAS CAN bus.

**Phase 1 only**: the carcontroller is a no-op. The safety model is `noOutput`.
This matches the priuspilot Phase 1 philosophy: observe and log, never actuate.

## Files to copy into retropilot

```
retropilot-port/
├── opendbc/
│   └── prius_gen2_pt.dbc          → opendbc/prius_gen2_pt.dbc
└── selfdrive/car/
    └── toyota_gen2/
        ├── __init__.py            → selfdrive/car/toyota_gen2/__init__.py
        ├── values.py              → selfdrive/car/toyota_gen2/values.py
        ├── carstate.py            → selfdrive/car/toyota_gen2/carstate.py
        ├── carcontroller.py       → selfdrive/car/toyota_gen2/carcontroller.py
        ├── interface.py           → selfdrive/car/toyota_gen2/interface.py
        └── radar_interface.py     → selfdrive/car/toyota_gen2/radar_interface.py
```

## CAN bus notes

- Bus speed: 500 kbit/s
- No ADAS bus — single CAN bus only (bus 0)
- No camera CAN — `get_cam_can_parser()` returns `None`
- Key addresses: steering angle 0x025, speed 0x3CA, gear 0x120

## How the fingerprint works

The Prius Gen 2 has a fixed set of messages that appear on the bus in READY mode.
Car identification uses the CAN address/length fingerprint in `values.py`. There
are no firmware version (FW_VERSIONS) checks since the car has no ADAS ECUs.

## Differences from the flowpilot (priuspilot) version

| Aspect | priuspilot (flowpilot) | retropilot (openpilot) |
|---|---|---|
| Conversions import | `common.conversions` | `selfdrive.config` |
| CarController return | `can_sends` | `(new_actuators, can_sends)` |
| Interface `apply()` args | flowpilot style | openpilot style |
| Safety model | `noOutput` | `noOutput` |
