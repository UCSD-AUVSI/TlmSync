#Telemetry Synchronization
Note: This is a stop-gap program designed for the 2014 initial system test.

##Purpose
This system will take in AHRS from the autopilot, gimbal angles, and output a shoot pulse and synchronized telemetry stream.

##Pinout
Use PF0(ADC0) for roll
Use PF1(ADC1) for pitch
Use PE0(RXD0) for MAVlink Rx
Use PE1(TXD0) for MAVlink Tx
Use PD2(RXD1) for OBC Rx
Use PD3(TXD1) for OBC Tx
Use PE4 for Shoot
Use PE5 for Focus (Always high)
