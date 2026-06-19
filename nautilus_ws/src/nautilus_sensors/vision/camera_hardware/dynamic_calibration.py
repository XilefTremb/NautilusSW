#!/usr/bin/env python3

import depthai as dai
import time
from pathlib import Path

BACKUP_PATH = "json_config/calibration_backup.json"
NEW_CALIB_PATH = "calibration_dynamic_new.json"

CHECK_INTERVAL = 3.0
SAMPSON_THRESHOLD = 0.05

STATE_COLLECTING = "COLLECTING"
STATE_CALIBRATING = "CALIBRATING"
STATE_DONE = "DONE"


def find_oakd_device():
    device_infos = dai.Device.getAllAvailableDevices()

    print(f"Found {len(device_infos)} device(s).")

    for device_info in device_infos:
        print(f"\nChecking device: {device_info}")

        try:
            with dai.Device(device_info) as device:
                cameras = device.getConnectedCameras()
                print(f"Connected cameras: {cameras}")

                if (
                    dai.CameraBoardSocket.CAM_B in cameras
                    and dai.CameraBoardSocket.CAM_C in cameras
                ):
                    print("OAK-D found.")
                    return device_info

        except Exception as e:
            print(f"Could not check device: {e}")

    print("No OAK-D found. Make sure the OAK-D is connected.")
    return None

with dai.Device(find_oakd_device()) as device:
    with dai.Pipeline(device) as pipeline:
        print(f"\nUsing OAK-D device: {device.getDeviceId()}")
        print(f"Connected cameras: {device.getConnectedCameras()}")

        monoLeft = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B
        )
        monoRight = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C
        )

        monoLeftOut = monoLeft.requestFullResolutionOutput()
        monoRightOut = monoRight.requestFullResolutionOutput()

        dynCalib = pipeline.create(dai.node.DynamicCalibration)

        monoLeftOut.link(dynCalib.left)
        monoRightOut.link(dynCalib.right)

        dynCalibCoverageQueue = dynCalib.coverageOutput.createOutputQueue()
        dynCalibQualityQueue = dynCalib.qualityOutput.createOutputQueue()
        dynCalibCalibrationQueue = dynCalib.calibrationOutput.createOutputQueue()
        dynCalibInputControl = dynCalib.inputControl.createInputQueue()

        device.setCalibration(device.readCalibration())

        pipeline.start()
        time.sleep(1)

        print("\nDynamic calibration started.")
        print("Move cam")
        print("Ctrl+C to quit.\n")

        state = STATE_COLLECTING
        last_check = time.time()

        try:
            while pipeline.isRunning() and state != STATE_DONE:
                coverage = dynCalibCoverageQueue.tryGet()

                if coverage is not None:
                    print(
                        f"Coverage: {coverage.meanCoverage:.1f}% | "
                        f"Data: {coverage.dataAcquired:.1f}%"
                    )

                if state == STATE_COLLECTING and time.time() - last_check > CHECK_INTERVAL:
                    dynCalibInputControl.send(
                        dai.DynamicCalibrationControl.loadImage()
                    )
                    dynCalibInputControl.send(
                        dai.DynamicCalibrationControl.calibrationQuality(True)
                    )
                    last_check = time.time()

                dynQualityResult = dynCalibQualityQueue.tryGet()

                if state == STATE_COLLECTING and dynQualityResult is not None:
                    print(f"Quality status: {dynQualityResult.info}")

                    if dynQualityResult.qualityData:
                        quality = dynQualityResult.qualityData

                        diff = abs(
                            quality.sampsonErrorNew
                            - quality.sampsonErrorCurrent
                        )

                        print("\nQuality evaluated:")
                        print(f"Current Sampson error: {quality.sampsonErrorCurrent:.3f} px")
                        print(f"New Sampson error:     {quality.sampsonErrorNew:.3f} px")
                        print(f"Difference:            {diff:.3f} px")

                        print(f"Depth error diff @1m:  {quality.depthErrorDifference[0]:.2f}%")
                        print(f"Depth error diff @2m:  {quality.depthErrorDifference[1]:.2f}%")
                        print(f"Depth error diff @5m:  {quality.depthErrorDifference[2]:.2f}%")
                        print(f"Depth error diff @10m: {quality.depthErrorDifference[3]:.2f}%")

                        if diff > SAMPSON_THRESHOLD:
                            print("\nStart recalibration process...")
                            dynCalibInputControl.send(
                                dai.DynamicCalibrationControl.startCalibration()
                            )
                            state = STATE_CALIBRATING
                        else:
                            print("\nDifference too small. No calibration started.")
                            dynCalibInputControl.send(
                                dai.DynamicCalibrationControl.resetData()
                            )

                dynCalibrationResult = dynCalibCalibrationQueue.tryGet()

                if state == STATE_CALIBRATING and dynCalibrationResult is not None:
                    print(f"\nCalibration status: {dynCalibrationResult.info}")

                    calibrationData = dynCalibrationResult.calibrationData

                    if calibrationData:
                        print("Successfully calibrated.")

                        device.readCalibration().eepromToJsonFile(BACKUP_PATH)
                        calibrationData.newCalibration.eepromToJsonFile(
                            NEW_CALIB_PATH
                        )

                        print(f"Backup saved: {Path(BACKUP_PATH).resolve()}")
                        print(f"New calibration saved: {Path(NEW_CALIB_PATH).resolve()}")

                        dynCalibInputControl.send(
                            dai.DynamicCalibrationControl.applyCalibration(
                                calibrationData.newCalibration
                            )
                        )

                        print("\nCalibration applied LIVE.")

                        answer = input(
                            "Flash this calibration permanently to EEPROM? [y/N]: "
                        ).strip().lower()

                        if answer in ["y", "yes"]:
                            try:
                                device.flashCalibration(
                                    calibrationData.newCalibration
                                )
                                print("Calibration permanently flashed to EEPROM.")
                            except Exception as e:
                                print(f"Flash failed: {e}")
                        else:
                            print("Calibration NOT flashed.")
                            print("Only this live session uses it.")

                        dynCalibInputControl.send(
                            dai.DynamicCalibrationControl.resetData()
                        )

                        state = STATE_DONE

                    else:
                        if "Not enough" in str(dynCalibrationResult.info):
                            print("Still collecting calibration data...")
                        else:
                            print("Calibration failed. Returning to collecting mode.")
                            dynCalibInputControl.send(
                                dai.DynamicCalibrationControl.resetData()
                            )
                            state = STATE_COLLECTING
                            last_check = time.time()

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopped by user.")

        finally:
            pipeline.stop()
            print("Pipeline stopped.")