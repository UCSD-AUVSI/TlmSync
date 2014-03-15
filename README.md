#Telemetry Synchronization
Telemetry Synchronization Setup for 2014 initial system test.

##Purpose
This system will take in AHRS from the autopilot, gimbal angles, and output a shoot pulse and synchronized telemetry stream.

##Pinout
Use PF0 for roll
Use PF1 for pitch
Use PA09 for MAVlink Rx
Use PA10 for MAVlink Tx (not implemented currently)
Use PB4 for OBC Rx
Use PD3 for OBC Tx
Use PE4 for Shoot
Use PE5 for Focus (Always high)
