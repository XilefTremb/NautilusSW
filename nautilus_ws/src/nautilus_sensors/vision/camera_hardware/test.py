#!/usr/bin/env python3

import json
import depthai as dai
from pathlib import Path

BACKUP_JSON = Path("calibration_backup.json")
CURRENT_JSON = Path("calibration_current_tmp.json")
MERGED_JSON = Path("calibration_restore_merged.json")


def find_oakd_device():
    for device_info in dai.Device.getAllAvailableDevices():
        try:
            with dai.Device(device_info) as device:
                cameras = device.getConnectedCameras()

                if (
                    dai.CameraBoardSocket.CAM_B in cameras
                    and dai.CameraBoardSocket.CAM_C in cameras
                ):
                    return device_info

        except Exception as e:
            print(f"Could not check device: {e}")

    return None


if not BACKUP_JSON.exists():
    raise FileNotFoundError(f"Missing backup file: {BACKUP_JSON}")

oakd_device_info = find_oakd_device()

if oakd_device_info is None:
    raise RuntimeError("No OAK-D found. Make sure the OAK-D is connected.")

with dai.Device(oakd_device_info) as device:
    print("OAK-D detected.")
    print(f"Device: {device.getDeviceId()}")

    # Save current EEPROM calibration, including protected fields
    device.readCalibration().eepromToJsonFile(str(CURRENT_JSON))

    with BACKUP_JSON.open("r") as f:
        backup = json.load(f)

    with CURRENT_JSON.open("r") as f:
        current = json.load(f)

    # Only copy calibration-related fields from backup
    fields_to_restore = [
        "cameraData",
        "stereoRectificationData",
        "stereoUseSpecTranslation",
        "stereoEnableDistortionCorrection",
        "imuExtrinsics",
        "housingExtrinsics",
        "verticalCameraSocket",
        "version",
    ]

    for field in fields_to_restore:
        if field in backup:
            current[field] = backup[field]

    with MERGED_JSON.open("w") as f:
        json.dump(current, f, indent=4)

    print(f"Merged restore file saved: {MERGED_JSON.resolve()}")

    answer = input(
        "\nThis will restore old calibration values while keeping current protected EEPROM fields.\n"
        "Flash this merged calibration to EEPROM? [y/N]: "
    ).strip().lower()

    if answer not in ["y", "yes"]:
        print("Flash cancelled.")
        exit(0)

    calib = dai.CalibrationHandler(str(MERGED_JSON))

    try:
        device.flashCalibration(calib)
        print("Calibration restored successfully.")
    except Exception as e:
        print(f"Flash failed: {e}")