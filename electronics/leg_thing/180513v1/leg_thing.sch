EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:leg_thing
LIBS:teensy
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L GND #PWR01
U 1 1 5AFA1C65
P 4050 1550
F 0 "#PWR01" H 4050 1300 50  0001 C CNN
F 1 "GND" H 4050 1400 50  0000 C CNN
F 2 "" H 4050 1550 50  0001 C CNN
F 3 "" H 4050 1550 50  0001 C CNN
	1    4050 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5AFA1C97
P 1550 3450
F 0 "#PWR02" H 1550 3200 50  0001 C CNN
F 1 "GND" H 1550 3300 50  0000 C CNN
F 2 "" H 1550 3450 50  0001 C CNN
F 3 "" H 1550 3450 50  0001 C CNN
	1    1550 3450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR03
U 1 1 5AFA1CD6
P 3950 3350
F 0 "#PWR03" H 3950 3200 50  0001 C CNN
F 1 "+5V" H 3950 3490 50  0000 C CNN
F 2 "" H 3950 3350 50  0001 C CNN
F 3 "" H 3950 3350 50  0001 C CNN
	1    3950 3350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 5AFA1D01
P 4050 3050
F 0 "#PWR04" H 4050 2900 50  0001 C CNN
F 1 "+3.3V" H 4050 3190 50  0000 C CNN
F 2 "" H 4050 3050 50  0001 C CNN
F 3 "" H 4050 3050 50  0001 C CNN
	1    4050 3050
	1    0    0    -1  
$EndComp
$Comp
L ESP32THINGPOWERSHIELD U1
U 1 1 5AFA345E
P 2750 2500
F 0 "U1" H 2750 2500 60  0000 C CNN
F 1 "ESP32THINGPOWERSHIELD" H 2750 2250 60  0000 C CNN
F 2 "leg_thing:ESP32THINGPOWERSHIELD" H 2750 2600 60  0001 C CNN
F 3 "" H 2750 2600 60  0001 C CNN
	1    2750 2500
	1    0    0    -1  
$EndComp
$Comp
L Teensy3.2_no_outer_inner U2
U 1 1 5AFA34BA
P 7800 2950
F 0 "U2" H 7800 4450 60  0000 C CNN
F 1 "Teensy3.2_no_outer_inner" H 7800 1450 60  0000 C CNN
F 2 "teensy:Teensy30_31_32_LC_NO_INNER_OUTER" H 7800 2150 60  0001 C CNN
F 3 "" H 7800 2150 60  0000 C CNN
	1    7800 2950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5AFA3549
P 4050 3450
F 0 "#PWR05" H 4050 3200 50  0001 C CNN
F 1 "GND" H 4050 3300 50  0000 C CNN
F 2 "" H 4050 3450 50  0001 C CNN
F 3 "" H 4050 3450 50  0001 C CNN
	1    4050 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1550 4050 1550
Wire Wire Line
	3800 3450 4050 3450
Wire Wire Line
	3800 3150 4050 3150
Wire Wire Line
	4050 3150 4050 3450
Wire Wire Line
	1750 3450 1550 3450
Wire Wire Line
	1750 3150 1550 3150
Wire Wire Line
	1550 3150 1550 3450
Wire Wire Line
	3800 3350 3950 3350
Wire Wire Line
	3800 3050 4050 3050
Wire Wire Line
	3800 2350 3900 2350
Wire Wire Line
	3800 2250 3900 2250
Wire Wire Line
	3800 2150 3900 2150
Wire Wire Line
	3800 2050 3900 2050
Wire Wire Line
	1750 1950 1400 1950
Wire Wire Line
	1750 2050 1400 2050
Wire Wire Line
	1750 2150 1400 2150
Wire Wire Line
	1750 1850 1400 1850
Wire Wire Line
	1750 1550 1400 1550
Wire Wire Line
	1750 2350 1400 2350
Text Label 1400 2350 0    60   ~ 0
HIP0
Text Label 1400 2150 0    60   ~ 0
FB2
Text Label 1400 2050 0    60   ~ 0
THIGH1
Text Label 1400 1950 0    60   ~ 0
KNEE0
Text Label 1400 1850 0    60   ~ 0
FB4
Text Label 1400 1550 0    60   ~ 0
FB6
Text Label 3900 2050 0    60   ~ 0
KNEE1
Text Label 3900 2150 0    60   ~ 0
THIGH0
Text Label 3900 2250 0    60   ~ 0
HIP1
Text Label 3900 2350 0    60   ~ 0
MDISABLE
$Comp
L GND #PWR06
U 1 1 5AFA3C38
P 5950 1650
F 0 "#PWR06" H 5950 1400 50  0001 C CNN
F 1 "GND" H 5950 1500 50  0000 C CNN
F 2 "" H 5950 1650 50  0001 C CNN
F 3 "" H 5950 1650 50  0001 C CNN
	1    5950 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 1650 5950 1650
$Comp
L +5V #PWR07
U 1 1 5AFA3C88
P 9000 3750
F 0 "#PWR07" H 9000 3600 50  0001 C CNN
F 1 "+5V" H 9000 3890 50  0000 C CNN
F 2 "" H 9000 3750 50  0001 C CNN
F 3 "" H 9000 3750 50  0001 C CNN
	1    9000 3750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR08
U 1 1 5AFA3CAD
P 9500 3950
F 0 "#PWR08" H 9500 3800 50  0001 C CNN
F 1 "+3.3V" H 9500 4090 50  0000 C CNN
F 2 "" H 9500 3950 50  0001 C CNN
F 3 "" H 9500 3950 50  0001 C CNN
	1    9500 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 3750 9000 3750
Wire Wire Line
	8800 3950 9500 3950
$Comp
L GNDA #PWR09
U 1 1 5AFA3E95
P 9250 3750
F 0 "#PWR09" H 9250 3500 50  0001 C CNN
F 1 "GNDA" H 9250 3600 50  0000 C CNN
F 2 "" H 9250 3750 50  0001 C CNN
F 3 "" H 9250 3750 50  0001 C CNN
	1    9250 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 3850 9100 3850
Wire Wire Line
	9100 3850 9100 3750
Wire Wire Line
	9100 3750 9250 3750
Wire Wire Line
	6800 1850 6250 1850
Text Label 6250 1850 0    60   ~ 0
MDISABLE
Wire Wire Line
	6800 1950 6250 1950
Text Label 6250 1950 0    60   ~ 0
M2EN
Wire Wire Line
	6800 2050 6250 2050
Text Label 6250 2050 0    60   ~ 0
THIGH0
Wire Wire Line
	6800 2150 6250 2150
Text Label 6250 2150 0    60   ~ 0
THIGH1
Wire Wire Line
	6800 2250 6250 2250
Wire Wire Line
	6800 2350 6250 2350
Text Label 6250 2250 0    60   ~ 0
KNEE0
Text Label 6250 2350 0    60   ~ 0
KNEE1
Wire Wire Line
	6800 2550 6250 2550
Text Label 6250 2550 0    60   ~ 0
M1EN
Wire Wire Line
	6800 2650 6250 2650
Text Label 6250 2650 0    60   ~ 0
HIP0
Wire Wire Line
	6800 2750 6250 2750
Text Label 6250 2750 0    60   ~ 0
HIP1
Wire Wire Line
	6800 3750 6250 3750
Wire Wire Line
	6800 3850 6250 3850
Wire Wire Line
	6800 3950 6250 3950
Text Label 6250 3750 0    60   ~ 0
KNEEPOT
Text Label 6250 3850 0    60   ~ 0
CALFPOT3V3
Text Label 6250 3950 0    60   ~ 0
THIGHPOT
Wire Wire Line
	6800 4250 6250 4250
Text Label 6250 4250 0    60   ~ 0
HIPPOT
$Comp
L R R1
U 1 1 5AFA461D
P 9700 5350
F 0 "R1" V 9780 5350 50  0000 C CNN
F 1 "R" V 9700 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9630 5350 50  0001 C CNN
F 3 "" H 9700 5350 50  0001 C CNN
	1    9700 5350
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5AFA4669
P 9700 5650
F 0 "R2" V 9780 5650 50  0000 C CNN
F 1 "R" V 9700 5650 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9630 5650 50  0001 C CNN
F 3 "" H 9700 5650 50  0001 C CNN
	1    9700 5650
	1    0    0    -1  
$EndComp
Connection ~ 9700 5500
Wire Wire Line
	9700 5500 10000 5500
Wire Wire Line
	9700 5200 9200 5200
Text Label 10000 5500 0    60   ~ 0
CALFPOT3V3
Text Label 9200 5200 0    60   ~ 0
CALFPOT5V
$Comp
L Screw_Terminal_01x03 J1
U 1 1 5AFA4AAE
P 3150 4450
F 0 "J1" H 3150 4650 50  0000 C CNN
F 1 "HIP" H 3150 4250 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-3_P5.08mm" H 3150 4450 50  0001 C CNN
F 3 "" H 3150 4450 50  0001 C CNN
	1    3150 4450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR010
U 1 1 5AFA4B8E
P 2800 4350
F 0 "#PWR010" H 2800 4200 50  0001 C CNN
F 1 "+3.3V" H 2800 4490 50  0000 C CNN
F 2 "" H 2800 4350 50  0001 C CNN
F 3 "" H 2800 4350 50  0001 C CNN
	1    2800 4350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR011
U 1 1 5AFA4C9D
P 2800 4550
F 0 "#PWR011" H 2800 4300 50  0001 C CNN
F 1 "GNDA" H 2800 4400 50  0000 C CNN
F 2 "" H 2800 4550 50  0001 C CNN
F 3 "" H 2800 4550 50  0001 C CNN
	1    2800 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4350 2950 4350
Wire Wire Line
	2800 4550 2950 4550
Wire Wire Line
	2950 4450 2550 4450
Text Label 2550 4450 0    60   ~ 0
HIPPOT
$Comp
L Screw_Terminal_01x03 J2
U 1 1 5AFA4F24
P 3150 5100
F 0 "J2" H 3150 5300 50  0000 C CNN
F 1 "THIGH" H 3150 4900 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-3_P5.08mm" H 3150 5100 50  0001 C CNN
F 3 "" H 3150 5100 50  0001 C CNN
	1    3150 5100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR012
U 1 1 5AFA4F2A
P 2800 5000
F 0 "#PWR012" H 2800 4850 50  0001 C CNN
F 1 "+3.3V" H 2800 5140 50  0000 C CNN
F 2 "" H 2800 5000 50  0001 C CNN
F 3 "" H 2800 5000 50  0001 C CNN
	1    2800 5000
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR013
U 1 1 5AFA4F30
P 2800 5200
F 0 "#PWR013" H 2800 4950 50  0001 C CNN
F 1 "GNDA" H 2800 5050 50  0000 C CNN
F 2 "" H 2800 5200 50  0001 C CNN
F 3 "" H 2800 5200 50  0001 C CNN
	1    2800 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 5000 2950 5000
Wire Wire Line
	2800 5200 2950 5200
Wire Wire Line
	2950 5100 2550 5100
Text Label 2550 5100 0    60   ~ 0
THIGHPOT
$Comp
L Screw_Terminal_01x03 J3
U 1 1 5AFA4F70
P 3150 5750
F 0 "J3" H 3150 5950 50  0000 C CNN
F 1 "KNEE" H 3150 5550 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-3_P5.08mm" H 3150 5750 50  0001 C CNN
F 3 "" H 3150 5750 50  0001 C CNN
	1    3150 5750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR014
U 1 1 5AFA4F76
P 2800 5650
F 0 "#PWR014" H 2800 5500 50  0001 C CNN
F 1 "+3.3V" H 2800 5790 50  0000 C CNN
F 2 "" H 2800 5650 50  0001 C CNN
F 3 "" H 2800 5650 50  0001 C CNN
	1    2800 5650
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR015
U 1 1 5AFA4F7C
P 2800 5850
F 0 "#PWR015" H 2800 5600 50  0001 C CNN
F 1 "GNDA" H 2800 5700 50  0000 C CNN
F 2 "" H 2800 5850 50  0001 C CNN
F 3 "" H 2800 5850 50  0001 C CNN
	1    2800 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 5650 2950 5650
Wire Wire Line
	2800 5850 2950 5850
Wire Wire Line
	2950 5750 2550 5750
Text Label 2550 5750 0    60   ~ 0
KNEEPOT
$Comp
L Screw_Terminal_01x03 J4
U 1 1 5AFA4FAB
P 3150 6400
F 0 "J4" H 3150 6600 50  0000 C CNN
F 1 "CALF" H 3150 6200 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-3_P5.08mm" H 3150 6400 50  0001 C CNN
F 3 "" H 3150 6400 50  0001 C CNN
	1    3150 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 6300 2950 6300
Wire Wire Line
	2800 6500 2950 6500
Wire Wire Line
	2500 6400 2950 6400
Text Label 2500 6400 0    60   ~ 0
CALFPOT5V
$Comp
L GND #PWR016
U 1 1 5AFA532B
P 9700 5800
F 0 "#PWR016" H 9700 5550 50  0001 C CNN
F 1 "GND" H 9700 5650 50  0000 C CNN
F 2 "" H 9700 5800 50  0001 C CNN
F 3 "" H 9700 5800 50  0001 C CNN
	1    9700 5800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5AFA5426
P 2800 6500
F 0 "#PWR017" H 2800 6250 50  0001 C CNN
F 1 "GND" H 2800 6350 50  0000 C CNN
F 2 "" H 2800 6500 50  0001 C CNN
F 3 "" H 2800 6500 50  0001 C CNN
	1    2800 6500
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR018
U 1 1 5AFA5531
P 2800 6300
F 0 "#PWR018" H 2800 6150 50  0001 C CNN
F 1 "+5V" H 2800 6440 50  0000 C CNN
F 2 "" H 2800 6300 50  0001 C CNN
F 3 "" H 2800 6300 50  0001 C CNN
	1    2800 6300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
