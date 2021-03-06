EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Simple PWM LED controller (3 channel)"
Date "2020-04-10"
Rev "2.0"
Comp "Copyright (c) 2018-2020 Michael Buesch <m@bues.ch>"
Comment1 "Licensed under the GNU/GPL v2 or (at your option) any later version."
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:AVR-ISP-6 J4
U 1 1 5E14DD07
P 4200 1150
F 0 "J4" V 3733 1200 50  0000 C CNN
F 1 "AVR-ISP-6" V 3824 1200 50  0000 C CNN
F 2 "" V 3950 1200 50  0001 C CNN
F 3 " ~" H 2925 600 50  0001 C CNN
	1    4200 1150
	0    1    1    0   
$EndComp
$Comp
L Device:R_POT RV3
U 1 1 5E14E8CB
P 6150 3850
F 0 "RV3" H 6080 3804 50  0000 R CNN
F 1 "100k" H 6080 3895 50  0000 R CNN
F 2 "" H 6150 3850 50  0001 C CNN
F 3 "~" H 6150 3850 50  0001 C CNN
	1    6150 3850
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5E14ECDB
P 1650 2300
F 0 "C2" V 1398 2300 50  0000 C CNN
F 1 "100n" V 1489 2300 50  0000 C CNN
F 2 "" H 1688 2150 50  0001 C CNN
F 3 "~" H 1650 2300 50  0001 C CNN
	1    1650 2300
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR03
U 1 1 5E153604
P 1900 2200
F 0 "#PWR03" H 1900 2050 50  0001 C CNN
F 1 "VCC" H 1917 2373 50  0000 C CNN
F 2 "" H 1900 2200 50  0001 C CNN
F 3 "" H 1900 2200 50  0001 C CNN
	1    1900 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5E153BD6
P 1900 5900
F 0 "#PWR04" H 1900 5650 50  0001 C CNN
F 1 "GND" H 1905 5727 50  0000 C CNN
F 2 "" H 1900 5900 50  0001 C CNN
F 3 "" H 1900 5900 50  0001 C CNN
	1    1900 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 5700 1900 5900
Wire Wire Line
	1900 2200 1900 2300
Wire Wire Line
	1800 2300 1900 2300
Connection ~ 1900 2300
Wire Wire Line
	1900 2300 1900 2700
$Comp
L power:GND #PWR02
U 1 1 5E1544B9
P 1300 2300
F 0 "#PWR02" H 1300 2050 50  0001 C CNN
F 1 "GND" V 1305 2172 50  0000 R CNN
F 2 "" H 1300 2300 50  0001 C CNN
F 3 "" H 1300 2300 50  0001 C CNN
	1    1300 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	1300 2300 1500 2300
$Comp
L power:VCC #PWR07
U 1 1 5E155655
P 4900 1050
F 0 "#PWR07" H 4900 900 50  0001 C CNN
F 1 "VCC" V 4917 1178 50  0000 L CNN
F 2 "" H 4900 1050 50  0001 C CNN
F 3 "" H 4900 1050 50  0001 C CNN
	1    4900 1050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5E1565B8
P 3600 1050
F 0 "#PWR05" H 3600 800 50  0001 C CNN
F 1 "GND" V 3605 922 50  0000 R CNN
F 2 "" H 3600 1050 50  0001 C CNN
F 3 "" H 3600 1050 50  0001 C CNN
	1    3600 1050
	0    1    1    0   
$EndComp
Wire Wire Line
	3600 1050 3800 1050
Wire Wire Line
	4700 1050 4900 1050
$Comp
L Device:R R1
U 1 1 5E1573BA
P 4100 4700
F 0 "R1" H 4170 4746 50  0000 L CNN
F 1 "10k" H 4170 4655 50  0000 L CNN
F 2 "" V 4030 4700 50  0001 C CNN
F 3 "~" H 4100 4700 50  0001 C CNN
	1    4100 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4500 4100 1550
Wire Wire Line
	4400 1550 4400 3400
Wire Wire Line
	4400 3400 2500 3400
$Comp
L power:VCC #PWR06
U 1 1 5E159838
P 4100 4900
F 0 "#PWR06" H 4100 4750 50  0001 C CNN
F 1 "VCC" H 4118 5073 50  0000 C CNN
F 2 "" H 4100 4900 50  0001 C CNN
F 3 "" H 4100 4900 50  0001 C CNN
	1    4100 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	4100 4850 4100 4900
$Comp
L Connector:Conn_01x01_Female J5
U 1 1 5E15A9D4
P 8050 4150
F 0 "J5" H 7900 4300 50  0000 L CNN
F 1 "PWM_OUTPUT" H 7700 4050 50  0000 L CNN
F 2 "" H 8050 4150 50  0001 C CNN
F 3 "~" H 8050 4150 50  0001 C CNN
	1    8050 4150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J8
U 1 1 5E177271
P 8200 4150
F 0 "J8" H 8300 4300 50  0000 C CNN
F 1 "Conn_01x01_Male" H 8200 4050 50  0001 C CNN
F 2 "" H 8200 4150 50  0001 C CNN
F 3 "~" H 8200 4150 50  0001 C CNN
	1    8200 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_DGS Q1
U 1 1 5E179C4B
P 8850 4150
F 0 "Q1" H 9054 4196 50  0000 L CNN
F 1 "Q_NMOS" H 9054 4105 50  0000 L CNN
F 2 "" H 9050 4250 50  0001 C CNN
F 3 "~" H 8850 4150 50  0001 C CNN
	1    8850 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5E17A678
P 8950 3450
F 0 "D1" V 8989 3333 50  0000 R CNN
F 1 "LED" V 8898 3333 50  0000 R CNN
F 2 "" H 8950 3450 50  0001 C CNN
F 3 "~" H 8950 3450 50  0001 C CNN
	1    8950 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5E17B11C
P 8950 2800
F 0 "R3" H 9020 2846 50  0000 L CNN
F 1 "R" H 9020 2755 50  0000 L CNN
F 2 "" V 8880 2800 50  0001 C CNN
F 3 "~" H 8950 2800 50  0001 C CNN
	1    8950 2800
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR08
U 1 1 5E17C118
P 8950 2300
F 0 "#PWR08" H 8750 2150 50  0001 C CNN
F 1 "Vdrive" H 8967 2473 50  0000 C CNN
F 2 "" H 8950 2300 50  0001 C CNN
F 3 "" H 8950 2300 50  0001 C CNN
	1    8950 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5E17C420
P 8950 4600
F 0 "#PWR09" H 8950 4350 50  0001 C CNN
F 1 "GND" H 8955 4427 50  0000 C CNN
F 2 "" H 8950 4600 50  0001 C CNN
F 3 "" H 8950 4600 50  0001 C CNN
	1    8950 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 4350 8950 4550
Wire Wire Line
	8400 4150 8550 4150
Wire Wire Line
	8950 3600 8950 3950
Wire Wire Line
	8950 2950 8950 3300
Wire Wire Line
	8950 2300 8950 2650
$Comp
L MCU_Microchip_ATmega:ATmega328P-PU U1
U 1 1 5E909D43
P 1900 4200
F 0 "U1" H 1257 4246 50  0000 R CNN
F 1 "ATmegaXXX" H 1257 4155 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 1900 4200 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 1900 4200 50  0001 C CNN
	1    1900 4200
	1    0    0    -1  
$EndComp
Text Notes 800  4500 0    50   ~ 0
ATMega328P
Wire Wire Line
	1900 2300 2000 2300
Wire Wire Line
	2000 2300 2000 2700
Wire Wire Line
	4300 1550 4300 3300
Wire Wire Line
	4300 3300 3000 3300
$Comp
L Connector:Conn_01x01_Male J9
U 1 1 5E94F59D
P 8200 4900
F 0 "J9" H 8300 5050 50  0000 C CNN
F 1 "Conn_01x01_Male" H 8200 4800 50  0001 C CNN
F 2 "" H 8200 4900 50  0001 C CNN
F 3 "~" H 8200 4900 50  0001 C CNN
	1    8200 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_DGS Q2
U 1 1 5E94F5A3
P 9650 4900
F 0 "Q2" H 9854 4946 50  0000 L CNN
F 1 "Q_NMOS" H 9854 4855 50  0000 L CNN
F 2 "" H 9850 5000 50  0001 C CNN
F 3 "~" H 9650 4900 50  0001 C CNN
	1    9650 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5E94F5A9
P 9750 3450
F 0 "D2" V 9789 3333 50  0000 R CNN
F 1 "LED" V 9698 3333 50  0000 R CNN
F 2 "" H 9750 3450 50  0001 C CNN
F 3 "~" H 9750 3450 50  0001 C CNN
	1    9750 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 5E94F5AF
P 9750 2800
F 0 "R5" H 9820 2846 50  0000 L CNN
F 1 "R" H 9820 2755 50  0000 L CNN
F 2 "" V 9680 2800 50  0001 C CNN
F 3 "~" H 9750 2800 50  0001 C CNN
	1    9750 2800
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR010
U 1 1 5E94F5B5
P 9750 2300
F 0 "#PWR010" H 9550 2150 50  0001 C CNN
F 1 "Vdrive" H 9767 2473 50  0000 C CNN
F 2 "" H 9750 2300 50  0001 C CNN
F 3 "" H 9750 2300 50  0001 C CNN
	1    9750 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5E94F5BB
P 9750 5350
F 0 "#PWR011" H 9750 5100 50  0001 C CNN
F 1 "GND" H 9755 5177 50  0000 C CNN
F 2 "" H 9750 5350 50  0001 C CNN
F 3 "" H 9750 5350 50  0001 C CNN
	1    9750 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 5100 9750 5300
Wire Wire Line
	8400 4900 9350 4900
Wire Wire Line
	9750 3600 9750 4700
Wire Wire Line
	9750 2950 9750 3300
Wire Wire Line
	9750 2300 9750 2650
$Comp
L Connector:Conn_01x01_Male J10
U 1 1 5E951D2C
P 8200 5650
F 0 "J10" H 8300 5800 50  0000 C CNN
F 1 "Conn_01x01_Male" H 8200 5550 50  0001 C CNN
F 2 "" H 8200 5650 50  0001 C CNN
F 3 "~" H 8200 5650 50  0001 C CNN
	1    8200 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_DGS Q3
U 1 1 5E951D32
P 10450 5650
F 0 "Q3" H 10654 5696 50  0000 L CNN
F 1 "Q_NMOS" H 10654 5605 50  0000 L CNN
F 2 "" H 10650 5750 50  0001 C CNN
F 3 "~" H 10450 5650 50  0001 C CNN
	1    10450 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5E951D38
P 10550 3450
F 0 "D3" V 10589 3333 50  0000 R CNN
F 1 "LED" V 10498 3333 50  0000 R CNN
F 2 "" H 10550 3450 50  0001 C CNN
F 3 "~" H 10550 3450 50  0001 C CNN
	1    10550 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R7
U 1 1 5E951D3E
P 10550 2800
F 0 "R7" H 10620 2846 50  0000 L CNN
F 1 "R" H 10620 2755 50  0000 L CNN
F 2 "" V 10480 2800 50  0001 C CNN
F 3 "~" H 10550 2800 50  0001 C CNN
	1    10550 2800
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR012
U 1 1 5E951D44
P 10550 2300
F 0 "#PWR012" H 10350 2150 50  0001 C CNN
F 1 "Vdrive" H 10567 2473 50  0000 C CNN
F 2 "" H 10550 2300 50  0001 C CNN
F 3 "" H 10550 2300 50  0001 C CNN
	1    10550 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5E951D4A
P 10550 6100
F 0 "#PWR013" H 10550 5850 50  0001 C CNN
F 1 "GND" H 10555 5927 50  0000 C CNN
F 2 "" H 10550 6100 50  0001 C CNN
F 3 "" H 10550 6100 50  0001 C CNN
	1    10550 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 5850 10550 6050
Wire Wire Line
	8400 5650 10150 5650
Wire Wire Line
	10550 2950 10550 3300
Wire Wire Line
	10550 2300 10550 2650
Wire Wire Line
	10550 3600 10550 5450
$Comp
L Connector:Conn_01x01_Female J6
U 1 1 5E97EE1E
P 8050 4900
F 0 "J6" H 7900 5050 50  0000 L CNN
F 1 "PWM_OUTPUT" H 7700 4800 50  0000 L CNN
F 2 "" H 8050 4900 50  0001 C CNN
F 3 "~" H 8050 4900 50  0001 C CNN
	1    8050 4900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J7
U 1 1 5E980565
P 8050 5650
F 0 "J7" H 7900 5800 50  0000 L CNN
F 1 "PWM_OUTPUT" H 7700 5550 50  0000 L CNN
F 2 "" H 8050 5650 50  0001 C CNN
F 3 "~" H 8050 5650 50  0001 C CNN
	1    8050 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT RV2
U 1 1 5E9829DE
P 6150 3100
F 0 "RV2" H 6080 3054 50  0000 R CNN
F 1 "100k" H 6080 3145 50  0000 R CNN
F 2 "" H 6150 3100 50  0001 C CNN
F 3 "~" H 6150 3100 50  0001 C CNN
	1    6150 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R_POT RV1
U 1 1 5E98306E
P 6150 2350
F 0 "RV1" H 6080 2304 50  0000 R CNN
F 1 "100k" H 6080 2395 50  0000 R CNN
F 2 "" H 6150 2350 50  0001 C CNN
F 3 "~" H 6150 2350 50  0001 C CNN
	1    6150 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 2350 6400 2350
Wire Wire Line
	6400 2350 6400 3100
Wire Wire Line
	6400 3850 6300 3850
Wire Wire Line
	6000 2350 5900 2350
Wire Wire Line
	5900 2350 5900 3100
Wire Wire Line
	5900 3850 6000 3850
Wire Wire Line
	5900 3100 6000 3100
Connection ~ 5900 3100
Wire Wire Line
	5900 3100 5900 3850
Wire Wire Line
	6300 3100 6400 3100
Connection ~ 6400 3100
Wire Wire Line
	6400 3100 6400 3850
Wire Wire Line
	4200 1550 4200 3500
Wire Wire Line
	2500 3500 4200 3500
Wire Wire Line
	5250 2600 6150 2600
Wire Wire Line
	6150 2600 6150 2500
Wire Wire Line
	2500 3900 5250 3900
Wire Wire Line
	6150 3250 6150 3350
Wire Wire Line
	6150 3350 5350 3350
Wire Wire Line
	5350 3350 5350 4000
Wire Wire Line
	2500 4000 5350 4000
Wire Wire Line
	6150 4100 6150 4000
Wire Wire Line
	2500 4300 5900 4300
Connection ~ 5900 3850
Wire Wire Line
	2500 4400 6400 4400
Connection ~ 6400 3850
NoConn ~ 2500 4200
Wire Wire Line
	2500 5300 7000 5300
Wire Wire Line
	7000 5300 7000 4150
Wire Wire Line
	7000 4150 7850 4150
Wire Wire Line
	2500 3100 3100 3100
Wire Wire Line
	3100 3100 3100 5400
Wire Wire Line
	3100 5400 7100 5400
Wire Wire Line
	7100 5400 7100 4900
Wire Wire Line
	7100 4900 7850 4900
Wire Wire Line
	3000 3300 3000 5500
Wire Wire Line
	3000 5500 7100 5500
Wire Wire Line
	7100 5500 7100 5650
Wire Wire Line
	7100 5650 7850 5650
Connection ~ 3000 3300
Wire Wire Line
	3000 3300 2500 3300
NoConn ~ 2500 3000
NoConn ~ 2500 3200
NoConn ~ 2500 3600
$Comp
L Connector:Conn_01x01_Female J1
U 1 1 5EAD3DBC
P 2100 6800
F 0 "J1" H 1950 6950 50  0000 L CNN
F 1 "DEBUG: Main loop indicator" H 1750 6700 50  0000 L CNN
F 2 "" H 2100 6800 50  0001 C CNN
F 3 "~" H 2100 6800 50  0001 C CNN
	1    2100 6800
	0    1    1    0   
$EndComp
NoConn ~ 2500 5000
NoConn ~ 2500 5100
NoConn ~ 2500 5200
NoConn ~ 2500 5400
Wire Wire Line
	2500 4800 2750 4800
Wire Wire Line
	2500 4900 2650 4900
Wire Wire Line
	2100 6150 2100 6600
$Comp
L Connector:Conn_01x01_Female J2
U 1 1 5EB0AFE8
P 2750 6800
F 0 "J2" H 2600 6950 50  0000 L CNN
F 1 "UART TxD" H 2400 6700 50  0000 L CNN
F 2 "" H 2750 6800 50  0001 C CNN
F 3 "~" H 2750 6800 50  0001 C CNN
	1    2750 6800
	0    1    1    0   
$EndComp
Wire Wire Line
	5250 3900 5250 2600
Wire Wire Line
	5900 4300 5900 3850
Wire Wire Line
	6400 4400 6400 3850
Wire Wire Line
	2500 4100 6150 4100
$Comp
L Device:C C1
U 1 1 5EB57783
P 1000 3250
F 0 "C1" H 1115 3296 50  0000 L CNN
F 1 "100n" H 1115 3205 50  0000 L CNN
F 2 "" H 1038 3100 50  0001 C CNN
F 3 "~" H 1000 3250 50  0001 C CNN
	1    1000 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 3000 1300 3000
$Comp
L power:GND #PWR01
U 1 1 5EB6681C
P 1000 3500
F 0 "#PWR01" H 1000 3250 50  0001 C CNN
F 1 "GND" V 1005 3372 50  0000 R CNN
F 2 "" H 1000 3500 50  0001 C CNN
F 3 "" H 1000 3500 50  0001 C CNN
	1    1000 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 3500 1000 3400
Wire Wire Line
	1000 3100 1000 3000
Text Notes 6550 2400 0    50   ~ 0
LED 0 control
Text Notes 6550 3150 0    50   ~ 0
LED 1 control
Text Notes 6550 3900 0    50   ~ 0
LED 2 control
Text Notes 8850 1900 0    50   ~ 0
LED 0
Text Notes 9650 1900 0    50   ~ 0
LED 1
Text Notes 10450 1900 0    50   ~ 0
LED 2
Wire Wire Line
	2500 4500 4100 4500
Wire Wire Line
	4100 4550 4100 4500
Connection ~ 4100 4500
Wire Wire Line
	2650 6150 2100 6150
Wire Wire Line
	2650 4900 2650 6150
NoConn ~ 2500 3700
$Comp
L Device:R R2
U 1 1 5E91D052
P 8550 4350
F 0 "R2" H 8480 4304 50  0000 R CNN
F 1 "10k" H 8480 4395 50  0000 R CNN
F 2 "" V 8480 4350 50  0001 C CNN
F 3 "~" H 8550 4350 50  0001 C CNN
	1    8550 4350
	-1   0    0    1   
$EndComp
Wire Wire Line
	8550 4200 8550 4150
Connection ~ 8550 4150
Wire Wire Line
	8550 4150 8650 4150
Wire Wire Line
	8550 4500 8550 4550
Wire Wire Line
	8550 4550 8950 4550
Connection ~ 8950 4550
Wire Wire Line
	8950 4550 8950 4600
$Comp
L Device:R R4
U 1 1 5E92E1E4
P 9350 5100
F 0 "R4" H 9280 5054 50  0000 R CNN
F 1 "10k" H 9280 5145 50  0000 R CNN
F 2 "" V 9280 5100 50  0001 C CNN
F 3 "~" H 9350 5100 50  0001 C CNN
	1    9350 5100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R6
U 1 1 5E92F1DD
P 10150 5850
F 0 "R6" H 10080 5804 50  0000 R CNN
F 1 "10k" H 10080 5895 50  0000 R CNN
F 2 "" V 10080 5850 50  0001 C CNN
F 3 "~" H 10150 5850 50  0001 C CNN
	1    10150 5850
	-1   0    0    1   
$EndComp
Wire Wire Line
	9350 4950 9350 4900
Connection ~ 9350 4900
Wire Wire Line
	9350 4900 9450 4900
Wire Wire Line
	9350 5250 9350 5300
Wire Wire Line
	9350 5300 9750 5300
Connection ~ 9750 5300
Wire Wire Line
	9750 5300 9750 5350
Wire Wire Line
	10150 5700 10150 5650
Connection ~ 10150 5650
Wire Wire Line
	10150 5650 10250 5650
Wire Wire Line
	10150 6000 10150 6050
Wire Wire Line
	10150 6050 10550 6050
Connection ~ 10550 6050
Wire Wire Line
	10550 6050 10550 6100
$Comp
L Connector:Conn_01x01_Female J3
U 1 1 5EB9E2A5
P 3200 6800
F 0 "J3" H 3050 6950 50  0000 L CNN
F 1 "UART RxD" H 2850 6700 50  0000 L CNN
F 2 "" H 3200 6800 50  0001 C CNN
F 3 "~" H 3200 6800 50  0001 C CNN
	1    3200 6800
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 4700 2850 4700
Wire Wire Line
	2850 4700 2850 6150
Wire Wire Line
	2850 6150 3200 6150
Wire Wire Line
	3200 6150 3200 6600
Wire Wire Line
	2750 4800 2750 6600
$EndSCHEMATC
