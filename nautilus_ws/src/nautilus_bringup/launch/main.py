from auv_pymavlink import AuvPymavlink
from math import pi

AUV = AuvPymavlink()

AUV.Connect()

AUV.Arm()

AUV.ChangeMode('GUIDED')

AUV.GoToWaypointLocal(1, 1, 1, 0)

AUV.GoToWaypointLocal(0, 0, 0.5, pi/2)

AUV.GoToWaypointLocal(1, 1, 1, 0)

AUV.GoToWaypointLocal(0, 0, 0.5, pi/2)
