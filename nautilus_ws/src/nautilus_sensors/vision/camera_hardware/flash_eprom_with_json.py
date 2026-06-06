#!/usr/bin/env python3

import depthai as dai
from pathlib import Path

CALIB_JSON = Path("calibration_restore_clean.json")



print(f"Calibration à flasher : {CALIB_JSON}")

if not CALIB_JSON.exists():
    raise FileNotFoundError(f"Fichier introuvable : {CALIB_JSON}")

answer = input(
    "\nATTENTION: Ceci va écraser la calibration actuellement dans l'EEPROM.\n"
    "Voulez-vous continuer ? [y/N]: "
).strip().lower()

if answer not in ["y", "yes"]:
    print("Opération annulée.")
    exit(0)

print("\nChargement du fichier de calibration...")

calib = dai.CalibrationHandler(str(CALIB_JSON))

with dai.Device() as device:
    print("Caméra détectée.")

    answer = input(
        "\nDernière confirmation.\n"
        "Flasher cette calibration dans l'EEPROM ? [y/N]: "
    ).strip().lower()

    if answer not in ["y", "yes"]:
        print("Flash annulé.")
        exit(0)

    try:
        success = device.flashCalibration(calib, flashProtected=False)

        print(f"\nRésultat du flash : {success}")
        print("Calibration flashée dans l'EEPROM avec succès.")

    except Exception as e:
        print(f"\nErreur durant le flash : {e}")