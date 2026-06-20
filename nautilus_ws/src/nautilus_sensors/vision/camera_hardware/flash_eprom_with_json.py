#!/usr/bin/env python3

from pathlib import Path
import depthai as dai
from camera_hardware.dynamic_calibration import find_oakd_device


CALIBRATION_FILE = "C:/Users/Xavier Lefebvre/Documents/GitHub/NautilusVision/scripts/Annotate_And_Save/calibration.json"

# Backup automatique
BACKUP_FILE = str((Path(__file__).parent / "json_config" / "depthai_calib_backup.json").resolve())

try:
    device = dai.Device(find_oakd_device())

    print("\n====================================")
    print("Connected Device")
    print("====================================")

    current_calib = device.readCalibration()
    current_calib.eepromToJsonFile(BACKUP_FILE)

    print("Current calibration backed up to:")
    print(BACKUP_FILE)
    print()

    answer = input(
        f"Flash calibration file:\n{CALIBRATION_FILE}\n\n"
        "Continue? (y/n): "
    ).strip().lower()

    if answer != "y":
        print("Operation cancelled.")
        exit(0)

    calib_data = dai.CalibrationHandler(CALIBRATION_FILE)
    print("\nFlashing calibration...")
    device.flashCalibration(calib_data)

    print("\nSuccessfully flashed calibration!")

except Exception as e:
    print("\nFailed flashing calibration:")
    print(e)