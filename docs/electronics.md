Each leg has a teensy 3.2 with a custom breakout PCB

Firmware
===
see: https://github.com/braingram/StompyLegControl/tree/bjg_reorg
current firmware: https://github.com/braingram/StompyLegControl/blob/bjg_reorg/examples/on_stompy_joystick_test/on_stompy_joystick_test.ino


Valve drivers
====
https://www.pololu.com/product/1213
https://www.pololu.com/product/1212

Voltage regulator
====
traco 2450

Joint sensors
====
String pots (1k)
JX-PA-15-N12-14S-132 and JX-PA-10-N12-14S-132
http://www.unimeasure.com/obj--pdf/jx-pa.pdf

Calf sensors
====
ER10031 Delphi Ride Height Sensor
http://www.summitracing.com/parts/dfp-er10031?seid=srese1&gclid=CjwKEAjw3_ypBRCwoKqKw5P9wgsSJAAbi2K94xzHsf7dmDgVCodPEyFrd_NVCg_UObi5ZNHQsuZ5GBoCj2fw_wcB

Pressure sensors (only on front right leg)
====
http://cdn.kempstoncontrols.com/files/703d895b12803d7cf655dd0d65ab0a7d/2200BGH3002A3UA.pdf

- Gems Sensors & Controls
- 2200 Series
- 4-20mA Output
- 0-3000 psi-gauge
- 1/4-18 M-NPT
- 4 PIN DIN (Micro) Mating Connector Supplied with flying leads (4-1/2˝ IP30) [only supplied on some??]
- Amplified Only RFI Protected CE Mark, UR
- .25%/1.5% Accuracy/Thermal

IMU (not currently connected)
====
model: mti-28a53g35
ros node: http://wiki.ros.org/xsens_driver
roslaunch xsens_driver xsens_driver.launch
