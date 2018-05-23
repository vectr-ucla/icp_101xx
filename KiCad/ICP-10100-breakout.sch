EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:icp-10100
LIBS:ICP-10100-breakout-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "TDK ICP-10100 Barometric Pressure Sensor Breakout"
Date "2018-05-22"
Rev "C"
Comp "Designed by Adrian Studer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ICP-10100 U1
U 1 1 5A823A37
P 2500 3500
F 0 "U1" V 2150 3850 60  0000 C CNN
F 1 "ICP-10100" H 2500 3500 60  0000 C CNN
F 2 "footprints:ICP-10100" H 2500 3500 60  0001 C CNN
F 3 "" H 2500 3500 60  0001 C CNN
F 4 "TDK" H 2500 3500 60  0001 C CNN "MFG Name"
F 5 "ICP-10100" H 2500 3500 60  0001 C CNN "MFG Part Num"
F 6 "Value" H 2500 3500 60  0001 C CNN "Dist PN"
F 7 "Value" H 2500 3500 60  0001 C CNN "Dist Link"
	1    2500 3500
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 5A823B64
P 3350 2900
F 0 "C1" H 3360 2970 50  0000 L CNN
F 1 "100nF" H 3360 2820 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3350 2900 50  0001 C CNN
F 3 "" H 3350 2900 50  0001 C CNN
F 4 "Value" H 3350 2900 60  0001 C CNN "Dist PN"
F 5 "Value" H 3350 2900 60  0001 C CNN "Dist Link"
	1    3350 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5A823B95
P 3350 3100
F 0 "#PWR01" H 3350 2850 50  0001 C CNN
F 1 "GND" H 3350 2950 50  0000 C CNN
F 2 "" H 3350 3100 50  0001 C CNN
F 3 "" H 3350 3100 50  0001 C CNN
	1    3350 3100
	1    0    0    -1  
$EndComp
$Comp
L FDG1024NZ Q1
U 1 1 5A824044
P 6350 3350
F 0 "Q1" H 6550 3425 50  0000 L CNN
F 1 "UM6K34NTCN" H 6550 3350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-363_SC-70-6" H 6550 3275 50  0001 L CIN
F 3 "" H 6350 3350 50  0001 L CNN
F 4 "ROHM" H 6350 3350 60  0001 C CNN "MFG Name"
F 5 "UM6K34NTCN" H 6350 3350 60  0001 C CNN "MFG Part Num"
F 6 "Value" H 6350 3350 60  0001 C CNN "Dist PN"
F 7 "Value" H 6350 3350 60  0001 C CNN "Dist Link"
	1    6350 3350
	0    1    1    0   
$EndComp
$Comp
L FDG1024NZ Q1
U 2 1 5A8240A4
P 6700 3600
F 0 "Q1" H 6900 3675 50  0000 L CNN
F 1 "UM6K34NTCN" H 6900 3600 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-363_SC-70-6" H 6900 3525 50  0001 L CIN
F 3 "" H 6700 3600 50  0001 L CNN
	2    6700 3600
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5A824421
P 6050 3100
F 0 "R2" V 6130 3100 50  0000 C CNN
F 1 "10K" V 6050 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5980 3100 50  0001 C CNN
F 3 "" H 6050 3100 50  0001 C CNN
F 4 "Value" H 6050 3100 60  0001 C CNN "Dist PN"
F 5 "Value" H 6050 3100 60  0001 C CNN "Dist Link"
	1    6050 3100
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5A824458
P 5850 3100
F 0 "R1" V 5930 3100 50  0000 C CNN
F 1 "10K" V 5850 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5780 3100 50  0001 C CNN
F 3 "" H 5850 3100 50  0001 C CNN
F 4 "Value" H 5850 3100 60  0001 C CNN "Dist PN"
F 5 "Value" H 5850 3100 60  0001 C CNN "Dist Link"
	1    5850 3100
	1    0    0    -1  
$EndComp
Text Label 5300 3250 0    60   ~ 0
SDA18
Text Label 5300 3350 0    60   ~ 0
SCL18
Text Label 8850 3450 2    60   ~ 0
SDA
Text Label 8850 3700 2    60   ~ 0
SCL
$Comp
L R R5
U 1 1 5A82469A
P 8400 3100
F 0 "R5" V 8480 3100 50  0000 C CNN
F 1 "10K" V 8400 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8330 3100 50  0001 C CNN
F 3 "" H 8400 3100 50  0001 C CNN
F 4 "Value" H 8400 3100 60  0001 C CNN "Dist PN"
F 5 "Value" H 8400 3100 60  0001 C CNN "Dist Link"
	1    8400 3100
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5A8246A0
P 8200 3100
F 0 "R4" V 8280 3100 50  0000 C CNN
F 1 "10K" V 8200 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8130 3100 50  0001 C CNN
F 3 "" H 8200 3100 50  0001 C CNN
F 4 "Value" H 8200 3100 60  0001 C CNN "Dist PN"
F 5 "Value" H 8200 3100 60  0001 C CNN "Dist Link"
	1    8200 3100
	1    0    0    -1  
$EndComp
$Comp
L AP1117-18 U2
U 1 1 5A8248FC
P 7300 2800
F 0 "U2" H 7150 2925 50  0000 C CNN
F 1 "AP7313R" H 7300 2925 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 7300 3000 50  0001 C CNN
F 3 "" H 7400 2550 50  0001 C CNN
F 4 "Diodes Inc" H 7300 2800 60  0001 C CNN "MFG Name"
F 5 "AP7313-18SRG-7" H 7300 2800 60  0001 C CNN "MFG Part Num"
F 6 "Value" H 7300 2800 60  0001 C CNN "Dist PN"
F 7 "Value" H 7300 2800 60  0001 C CNN "Dist Link"
	1    7300 2800
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5A824B35
P 7300 3150
F 0 "#PWR02" H 7300 2900 50  0001 C CNN
F 1 "GND" H 7300 3000 50  0000 C CNN
F 2 "" H 7300 3150 50  0001 C CNN
F 3 "" H 7300 3150 50  0001 C CNN
	1    7300 3150
	1    0    0    -1  
$EndComp
Text Label 8850 2800 2    60   ~ 0
VDD
$Comp
L C_Small C2
U 1 1 5A824C7A
P 6900 2950
F 0 "C2" H 6910 3020 50  0000 L CNN
F 1 "1uF" H 6910 2870 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6900 2950 50  0001 C CNN
F 3 "" H 6900 2950 50  0001 C CNN
F 4 "Value" H 6900 2950 60  0001 C CNN "Dist PN"
F 5 "Value" H 6900 2950 60  0001 C CNN "Dist Link"
	1    6900 2950
	1    0    0    -1  
$EndComp
$Comp
L C_Small C3
U 1 1 5A824D11
P 7700 2950
F 0 "C3" H 7710 3020 50  0000 L CNN
F 1 "1uF" H 7710 2870 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7700 2950 50  0001 C CNN
F 3 "" H 7700 2950 50  0001 C CNN
F 4 "Value" H 7700 2950 60  0001 C CNN "Dist PN"
F 5 "Value" H 7700 2950 60  0001 C CNN "Dist Link"
	1    7700 2950
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5A824D91
P 7950 2800
F 0 "R3" V 8030 2800 50  0000 C CNN
F 1 "1K" V 7950 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7880 2800 50  0001 C CNN
F 3 "" H 7950 2800 50  0001 C CNN
F 4 "Value" H 7950 2800 60  0001 C CNN "Dist PN"
F 5 "Value" H 7950 2800 60  0001 C CNN "Dist Link"
	1    7950 2800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR03
U 1 1 5A824E10
P 6900 3150
F 0 "#PWR03" H 6900 2900 50  0001 C CNN
F 1 "GND" H 6900 3000 50  0000 C CNN
F 2 "" H 6900 3150 50  0001 C CNN
F 3 "" H 6900 3150 50  0001 C CNN
	1    6900 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 5A824E3C
P 7700 3150
F 0 "#PWR04" H 7700 2900 50  0001 C CNN
F 1 "GND" H 7700 3000 50  0000 C CNN
F 2 "" H 7700 3150 50  0001 C CNN
F 3 "" H 7700 3150 50  0001 C CNN
	1    7700 3150
	1    0    0    -1  
$EndComp
Text Label 5300 3150 0    60   ~ 0
VDD18
$Comp
L GND #PWR05
U 1 1 5A8255A1
P 3100 3400
F 0 "#PWR05" H 3100 3150 50  0001 C CNN
F 1 "GND" H 3100 3250 50  0000 C CNN
F 2 "" H 3100 3400 50  0001 C CNN
F 3 "" H 3100 3400 50  0001 C CNN
	1    3100 3400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5A8256ED
P 2400 4200
F 0 "#PWR06" H 2400 3950 50  0001 C CNN
F 1 "GND" H 2400 4050 50  0000 C CNN
F 2 "" H 2400 4200 50  0001 C CNN
F 3 "" H 2400 4200 50  0001 C CNN
	1    2400 4200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J1
U 1 1 5A8259B9
P 9600 2900
F 0 "J1" H 9600 3100 50  0000 C CNN
F 1 "Conn_01x04" H 9600 2600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 9600 2900 50  0001 C CNN
F 3 "" H 9600 2900 50  0001 C CNN
F 4 "Value" H 9600 2900 60  0001 C CNN "Dist PN"
F 5 "Value" H 9600 2900 60  0001 C CNN "Dist Link"
	1    9600 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5A825B92
P 9300 3350
F 0 "#PWR07" H 9300 3100 50  0001 C CNN
F 1 "GND" H 9300 3200 50  0000 C CNN
F 2 "" H 9300 3350 50  0001 C CNN
F 3 "" H 9300 3350 50  0001 C CNN
	1    9300 3350
	1    0    0    -1  
$EndComp
$Comp
L Ferrite_Bead_Small L1
U 1 1 5A827627
P 3550 2700
F 0 "L1" H 3625 2750 50  0000 L CNN
F 1 "Ferrite" H 3625 2650 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 3480 2700 50  0001 C CNN
F 3 "" H 3550 2700 50  0001 C CNN
F 4 "Value" H 3550 2700 60  0001 C CNN "Dist PN"
F 5 "Value" H 3550 2700 60  0001 C CNN "Dist Link"
	1    3550 2700
	0    -1   -1   0   
$EndComp
$Comp
L Conn_01x04 J3
U 1 1 5A963DB9
P 4400 3250
F 0 "J3" H 4400 3450 50  0000 C CNN
F 1 "Conn_01x04" H 4400 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4400 3250 50  0001 C CNN
F 3 "" H 4400 3250 50  0001 C CNN
F 4 "Value" H 4400 3250 60  0001 C CNN "Dist PN"
F 5 "Value" H 4400 3250 60  0001 C CNN "Dist Link"
	1    4400 3250
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J2
U 1 1 5A963E14
P 5050 3250
F 0 "J2" H 5050 3450 50  0000 C CNN
F 1 "Conn_01x04" H 5050 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 5050 3250 50  0001 C CNN
F 3 "" H 5050 3250 50  0001 C CNN
F 4 "Value" H 5050 3250 60  0001 C CNN "Dist PN"
F 5 "Value" H 5050 3250 60  0001 C CNN "Dist Link"
	1    5050 3250
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5A964338
P 5450 3800
F 0 "#PWR08" H 5450 3550 50  0001 C CNN
F 1 "GND" H 5450 3650 50  0000 C CNN
F 2 "" H 5450 3800 50  0001 C CNN
F 3 "" H 5450 3800 50  0001 C CNN
	1    5450 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5A96436C
P 3800 3800
F 0 "#PWR09" H 3800 3550 50  0001 C CNN
F 1 "GND" H 3800 3650 50  0000 C CNN
F 2 "" H 3800 3800 50  0001 C CNN
F 3 "" H 3800 3800 50  0001 C CNN
	1    3800 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3450 5850 3250
Connection ~ 5850 3450
Wire Wire Line
	6050 3250 6050 3700
Connection ~ 6050 3700
Wire Wire Line
	6550 3450 9050 3450
Wire Wire Line
	6900 3700 9150 3700
Wire Wire Line
	6650 2800 6650 3400
Wire Wire Line
	5850 2950 5850 2800
Connection ~ 5850 2800
Wire Wire Line
	6050 2800 6050 2950
Connection ~ 6050 2800
Wire Wire Line
	6300 2800 6300 3150
Connection ~ 6300 2800
Wire Wire Line
	8200 2950 8200 2800
Wire Wire Line
	8400 2800 8400 2950
Wire Wire Line
	8200 3250 8200 3450
Connection ~ 8200 3450
Wire Wire Line
	8400 3250 8400 3700
Connection ~ 8400 3700
Wire Wire Line
	8100 2800 9400 2800
Connection ~ 8400 2800
Connection ~ 8200 2800
Connection ~ 6650 2800
Wire Wire Line
	7300 3150 7300 3100
Wire Wire Line
	7600 2800 7800 2800
Wire Wire Line
	7700 2850 7700 2800
Connection ~ 7700 2800
Wire Wire Line
	7700 3050 7700 3150
Wire Wire Line
	6900 3050 6900 3150
Wire Wire Line
	6900 2800 6900 2850
Connection ~ 6900 2800
Wire Wire Line
	3350 2700 3350 2800
Wire Wire Line
	3350 3000 3350 3100
Wire Wire Line
	3600 3250 3600 4400
Wire Wire Line
	9400 2900 9050 2900
Wire Wire Line
	9050 2900 9050 3450
Wire Wire Line
	9400 3000 9150 3000
Wire Wire Line
	9150 3000 9150 3700
Wire Wire Line
	9400 3100 9300 3100
Wire Wire Line
	9300 3100 9300 3350
Wire Wire Line
	5650 2800 7000 2800
Wire Wire Line
	5650 2800 5650 3150
Wire Wire Line
	5650 3150 5250 3150
Wire Wire Line
	5650 3450 6150 3450
Wire Wire Line
	5650 3450 5650 3250
Wire Wire Line
	5650 3250 5250 3250
Wire Wire Line
	5250 3350 5550 3350
Wire Wire Line
	5550 3350 5550 3700
Wire Wire Line
	5550 3700 6500 3700
Wire Wire Line
	5450 3450 5450 3800
Wire Wire Line
	3800 3150 4200 3150
Wire Wire Line
	3600 3250 4200 3250
Wire Wire Line
	3700 4500 3700 3350
Wire Wire Line
	3700 3350 4200 3350
Wire Wire Line
	4200 3450 3800 3450
Wire Wire Line
	3800 3450 3800 3800
Text Label 4150 3250 2    60   ~ 0
SDA18
Text Label 4150 3150 2    60   ~ 0
VDD18
Text Label 4150 3350 2    60   ~ 0
SCL18
Wire Wire Line
	5250 3450 5450 3450
$Comp
L Conn_01x04 J4
U 1 1 5A964E22
P 4800 4850
F 0 "J4" H 4800 5050 50  0000 C CNN
F 1 "Conn_01x04" H 4800 4550 50  0000 C CNN
F 2 "footprints:Bites" H 4800 4850 50  0001 C CNN
F 3 "" H 4800 4850 50  0001 C CNN
F 4 "Value" H 4800 4850 60  0001 C CNN "Dist PN"
F 5 "Value" H 4800 4850 60  0001 C CNN "Dist Link"
	1    4800 4850
	1    0    0    -1  
$EndComp
Text Label 4600 4750 2    60   ~ 0
VDD18
Text Label 4600 4850 2    60   ~ 0
SDA18
Text Label 4600 4950 2    60   ~ 0
SCL18
$Comp
L GND #PWR?
U 1 1 5A964FF9
P 4500 5100
F 0 "#PWR?" H 4500 4850 50  0001 C CNN
F 1 "GND" H 4500 4950 50  0000 C CNN
F 2 "" H 4500 5100 50  0001 C CNN
F 3 "" H 4500 5100 50  0001 C CNN
	1    4500 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 5050 4500 5050
Wire Wire Line
	4500 5050 4500 5100
Wire Wire Line
	2500 4100 2500 4400
Wire Wire Line
	2500 4400 3600 4400
Wire Wire Line
	3700 4500 1900 4500
Wire Wire Line
	1900 4500 1900 3600
Wire Wire Line
	1900 3600 2000 3600
Wire Wire Line
	2600 4100 2600 4200
Wire Wire Line
	2600 4200 3250 4200
Wire Wire Line
	2400 4200 2400 4100
Wire Wire Line
	2500 2900 2500 2800
Wire Wire Line
	2600 2900 2600 2800
Wire Wire Line
	2500 2800 3100 2800
Wire Wire Line
	3100 2800 3100 3400
Connection ~ 2600 2800
Wire Wire Line
	3000 3400 3000 3300
Wire Wire Line
	3000 3300 3100 3300
Connection ~ 3100 3300
Wire Wire Line
	3000 3600 3250 3600
Wire Wire Line
	3250 4200 3250 2700
Wire Wire Line
	3800 3150 3800 2700
Wire Wire Line
	3800 2700 3650 2700
Connection ~ 3350 2700
Wire Wire Line
	2400 2700 2400 2900
Connection ~ 3250 2700
Wire Wire Line
	1650 3400 2000 3400
Connection ~ 3250 3600
Wire Wire Line
	2400 2700 3450 2700
Wire Wire Line
	1650 3400 1650 3550
$Comp
L GND #PWR?
U 1 1 5A9B9686
P 1650 3550
F 0 "#PWR?" H 1650 3300 50  0001 C CNN
F 1 "GND" H 1650 3400 50  0000 C CNN
F 2 "" H 1650 3550 50  0001 C CNN
F 3 "" H 1650 3550 50  0001 C CNN
	1    1650 3550
	1    0    0    -1  
$EndComp
$EndSCHEMATC
