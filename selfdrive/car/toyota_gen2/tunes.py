#!/usr/bin/env python3
# PriusPilot — Prius Gen 2 ocelot lateral and longitudinal tunes
# Adapted from RetroPilot retropilot/tunes.py
from enum import Enum


class LongTunes(Enum):
  PEDAL = 0     # gas interceptor (pedal interceptor)
  ACTUATOR = 1  # direct throttle actuator


class LatTunes(Enum):
  PID_A = 1  # baseline PID for Prius Gen 2 wheelbase / steer ratio


# ── Longitudinal ─────────────────────────────────────────────────────────────

def set_long_tune(tune, name):
  if name == LongTunes.PEDAL:
    # Tuning for gas-pedal interceptor (softer, more damping at low speed)
    tune.deadzoneBP = [0., 8.05]
    tune.deadzoneV = [.0, .14]
    tune.kpBP = [0., 5., 20.]
    tune.kpV = [1.3, 1.0, 0.7]
    tune.kiBP = [0., 5., 12., 20., 27.]
    tune.kiV = [.35, .23, .20, .17, .1]
  elif name == LongTunes.ACTUATOR:
    # Tuning for direct throttle actuator (stiffer at low speed)
    tune.deadzoneBP = [0., 9.]
    tune.deadzoneV = [0., .75]
    tune.kpBP = [0., 5., 35.]
    tune.kiBP = [0., 35.]
    tune.kpV = [3.6, 2.4, 1.5]
    tune.kiV = [0.54, 0.36]
  else:
    raise NotImplementedError(f'Unknown LongTune: {name}')


# ── Lateral ──────────────────────────────────────────────────────────────────

def set_lat_tune(tune, name):
  tune.init('pid')
  tune.pid.kiBP = [0.0]
  tune.pid.kpBP = [0.0]
  if name == LatTunes.PID_A:
    # Gentle baseline — suitable for Prius Gen 2 steer ratio 17.1 with interceptor
    tune.pid.kpV = [0.05]
    tune.pid.kiV = [0.05]
    tune.pid.kf = 0.00003
  else:
    raise NotImplementedError(f'Unknown LatTune: {name}')
