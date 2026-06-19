#!/usr/bin/env python3

import depthai as dai
from camera_hardware.dynamic_calibration import find_oakd_device

device = dai.Device(find_oakd_device())
try:
    device.factoryResetCalibration()
    print(f'Factory reset calibration OK')
except Exception as ex:
    print(f'Factory reset calibration FAIL: {ex}')
