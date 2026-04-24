#!/usr/bin/env python3
# Prius Gen 2 (NHW20, 2004-2009) radar interface stub for retropilot (openpilot)
# Gen 2 has no factory radar — always returns no data.
from selfdrive.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

  def update(self, can_strings):
    return super().update(None)
