EESchema Schematic File Version 4
EELAYER 30 0
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
L Micromouse-rescue:STM32F411RETx-MCU_ST_STM32F4 U3
U 1 1 5CA58D82
P 5700 3900
F 0 "U3" H 5700 3200 50  0000 C CNN
F 1 "STM32F411RET6" H 5700 3050 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 5100 2200 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00115249.pdf" H 5700 3900 50  0001 C CNN
	1    5700 3900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR022
U 1 1 5CAA7639
P 8650 950
F 0 "#PWR022" H 8650 800 50  0001 C CNN
F 1 "+3.3V" H 8665 1123 50  0000 C CNN
F 2 "" H 8650 950 50  0001 C CNN
F 3 "" H 8650 950 50  0001 C CNN
	1    8650 950 
	1    0    0    -1  
$EndComp
Text Label 8700 1200 2    50   ~ 0
SWCLK
Text Label 6400 3800 0    50   ~ 0
SWCLK
Text Label 6400 3700 0    50   ~ 0
SWDIO
Text Label 8700 1100 2    50   ~ 0
SWDIO
$Comp
L Connector:Conn_01x04_Male J6
U 1 1 5CAA382E
P 8900 1200
F 0 "J6" H 8872 1082 50  0000 R CNN
F 1 "SWD" H 8872 1173 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8900 1200 50  0001 C CNN
F 3 "~" H 8900 1200 50  0001 C CNN
	1    8900 1200
	-1   0    0    1   
$EndComp
Wire Wire Line
	8700 1000 8650 1000
Wire Wire Line
	8650 1000 8650 950 
Wire Wire Line
	8700 1300 8650 1300
Wire Wire Line
	8650 1300 8650 1350
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5CABFA07
P 700 950
F 0 "J1" H 808 1131 50  0000 C CNN
F 1 "POWER_IN" H 808 1040 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 700 950 50  0001 C CNN
F 3 "~" H 700 950 50  0001 C CNN
	1    700  950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5CAC1279
P 1150 1100
F 0 "C1" H 1242 1146 50  0000 L CNN
F 1 "100nF" H 1242 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 1150 1100 50  0001 C CNN
F 3 "~" H 1150 1100 50  0001 C CNN
	1    1150 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5CAC279C
P 2400 1100
F 0 "C2" H 2492 1146 50  0000 L CNN
F 1 "100nF" H 2492 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2400 1100 50  0001 C CNN
F 3 "~" H 2400 1100 50  0001 C CNN
	1    2400 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  950  1150 950 
Wire Wire Line
	1150 950  1150 1000
Wire Wire Line
	2250 950  2400 950 
Wire Wire Line
	2400 950  2400 1000
Wire Wire Line
	900  1050 1000 1050
Wire Wire Line
	1000 1050 1000 1300
Wire Wire Line
	2400 1300 2400 1200
Wire Wire Line
	1150 1200 1150 1300
$Comp
L power:+3.3V #PWR012
U 1 1 5CAC388A
P 2600 850
F 0 "#PWR012" H 2600 700 50  0001 C CNN
F 1 "+3.3V" H 2615 1023 50  0000 C CNN
F 2 "" H 2600 850 50  0001 C CNN
F 3 "" H 2600 850 50  0001 C CNN
	1    2600 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 950  2600 950 
Wire Wire Line
	2600 950  2600 850 
Connection ~ 2400 950 
$Comp
L Regulator_Linear:AMS1117-3.3 U2
U 1 1 5CABD8F1
P 1950 950
F 0 "U2" H 1950 1192 50  0000 C CNN
F 1 "AMS1117-3.3" H 1950 1101 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 1950 1150 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 2050 700 50  0001 C CNN
	1    1950 950 
	1    0    0    -1  
$EndComp
Connection ~ 1150 950 
Wire Wire Line
	2400 1300 2600 1300
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5CACBE4E
P 1850 2000
F 0 "#FLG03" H 1850 2075 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 2173 50  0000 C CNN
F 2 "" H 1850 2000 50  0001 C CNN
F 3 "~" H 1850 2000 50  0001 C CNN
	1    1850 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2000 1850 2100
Wire Wire Line
	1000 1300 1150 1300
$Comp
L power:GND #PWR023
U 1 1 5CACE3A6
P 8650 1350
F 0 "#PWR023" H 8650 1100 50  0001 C CNN
F 1 "GND" H 8655 1177 50  0000 C CNN
F 2 "" H 8650 1350 50  0001 C CNN
F 3 "" H 8650 1350 50  0001 C CNN
	1    8650 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5CAD0B80
P 2600 1450
F 0 "#PWR013" H 2600 1200 50  0001 C CNN
F 1 "GND" H 2605 1277 50  0000 C CNN
F 2 "" H 2600 1450 50  0001 C CNN
F 3 "" H 2600 1450 50  0001 C CNN
	1    2600 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1450 2600 1300
Wire Wire Line
	2400 1300 1950 1300
Connection ~ 2400 1300
Connection ~ 1150 1300
Wire Wire Line
	1950 1250 1950 1300
Connection ~ 1950 1300
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5CAD44C2
P 1150 1400
F 0 "#FLG02" H 1150 1475 50  0001 C CNN
F 1 "PWR_FLAG" H 1150 1573 50  0000 C CNN
F 2 "" H 1150 1400 50  0001 C CNN
F 3 "~" H 1150 1400 50  0001 C CNN
	1    1150 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1150 1400 1150 1300
Wire Wire Line
	5500 5700 5500 5800
Wire Wire Line
	5500 5800 5600 5800
Wire Wire Line
	5900 5800 5900 5700
Wire Wire Line
	5800 5700 5800 5800
Connection ~ 5800 5800
Wire Wire Line
	5800 5800 5900 5800
Wire Wire Line
	5700 5700 5700 5800
Connection ~ 5700 5800
Wire Wire Line
	5700 5800 5800 5800
Wire Wire Line
	5600 5700 5600 5800
Connection ~ 5600 5800
Wire Wire Line
	5600 5800 5700 5800
$Comp
L power:GND #PWR017
U 1 1 5CADC973
P 5700 5900
F 0 "#PWR017" H 5700 5650 50  0001 C CNN
F 1 "GND" H 5705 5727 50  0000 C CNN
F 2 "" H 5700 5900 50  0001 C CNN
F 3 "" H 5700 5900 50  0001 C CNN
	1    5700 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 5900 5700 5800
Wire Wire Line
	5500 2200 5500 2050
Wire Wire Line
	5500 2050 5600 2050
Wire Wire Line
	6000 2050 6000 2200
Wire Wire Line
	5900 2200 5900 2050
Connection ~ 5900 2050
Wire Wire Line
	5900 2050 6000 2050
Wire Wire Line
	5800 2200 5800 2050
Connection ~ 5800 2050
Wire Wire Line
	5800 2050 5900 2050
Wire Wire Line
	5700 2200 5700 2050
Connection ~ 5700 2050
Wire Wire Line
	5700 2050 5750 2050
Wire Wire Line
	5600 2200 5600 2050
Connection ~ 5600 2050
Wire Wire Line
	5600 2050 5700 2050
$Comp
L power:+3.3V #PWR018
U 1 1 5CADFEAE
P 5750 2000
F 0 "#PWR018" H 5750 1850 50  0001 C CNN
F 1 "+3.3V" H 5765 2173 50  0000 C CNN
F 2 "" H 5750 2000 50  0001 C CNN
F 3 "" H 5750 2000 50  0001 C CNN
	1    5750 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 2000 5750 2050
Connection ~ 5750 2050
Wire Wire Line
	5750 2050 5800 2050
$Comp
L Connector:Conn_01x06_Female J9
U 1 1 5CAE1374
P 10300 1400
F 0 "J9" H 10328 1376 50  0000 L CNN
F 1 "Tof_front_right" H 10328 1285 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 10300 1400 50  0001 C CNN
F 3 "~" H 10300 1400 50  0001 C CNN
	1    10300 1400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J10
U 1 1 5CAE2A65
P 10300 2450
F 0 "J10" H 10328 2426 50  0000 L CNN
F 1 "Tof_front_left" H 10328 2335 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 10300 2450 50  0001 C CNN
F 3 "~" H 10300 2450 50  0001 C CNN
	1    10300 2450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J11
U 1 1 5CAE3E32
P 10300 3500
F 0 "J11" H 10328 3476 50  0000 L CNN
F 1 "ToF_right" H 10328 3385 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 10300 3500 50  0001 C CNN
F 3 "~" H 10300 3500 50  0001 C CNN
	1    10300 3500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J12
U 1 1 5CAE4FBC
P 10300 4550
F 0 "J12" H 10328 4526 50  0000 L CNN
F 1 "ToF_left" H 10328 4435 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 10300 4550 50  0001 C CNN
F 3 "~" H 10300 4550 50  0001 C CNN
	1    10300 4550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J13
U 1 1 5CAE908E
P 10350 5900
F 0 "J13" H 10378 5876 50  0000 L CNN
F 1 "Tof_front" H 10378 5785 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 10350 5900 50  0001 C CNN
F 3 "~" H 10350 5900 50  0001 C CNN
	1    10350 5900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR029
U 1 1 5CAE991A
P 10050 1150
F 0 "#PWR029" H 10050 1000 50  0001 C CNN
F 1 "+3.3V" H 10065 1323 50  0000 C CNN
F 2 "" H 10050 1150 50  0001 C CNN
F 3 "" H 10050 1150 50  0001 C CNN
	1    10050 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR030
U 1 1 5CAEA5BA
P 10050 2200
F 0 "#PWR030" H 10050 2050 50  0001 C CNN
F 1 "+3.3V" H 10065 2373 50  0000 C CNN
F 2 "" H 10050 2200 50  0001 C CNN
F 3 "" H 10050 2200 50  0001 C CNN
	1    10050 2200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR031
U 1 1 5CAEB065
P 10050 3250
F 0 "#PWR031" H 10050 3100 50  0001 C CNN
F 1 "+3.3V" H 10065 3423 50  0000 C CNN
F 2 "" H 10050 3250 50  0001 C CNN
F 3 "" H 10050 3250 50  0001 C CNN
	1    10050 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR032
U 1 1 5CAEBB0B
P 10050 4300
F 0 "#PWR032" H 10050 4150 50  0001 C CNN
F 1 "+3.3V" H 10065 4473 50  0000 C CNN
F 2 "" H 10050 4300 50  0001 C CNN
F 3 "" H 10050 4300 50  0001 C CNN
	1    10050 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 1200 10050 1200
Wire Wire Line
	10050 1200 10050 1150
Wire Wire Line
	10100 2250 10050 2250
Wire Wire Line
	10050 2250 10050 2200
Wire Wire Line
	10100 3300 10050 3300
Wire Wire Line
	10050 3300 10050 3250
Wire Wire Line
	10100 4350 10050 4350
Wire Wire Line
	10050 4350 10050 4300
$Comp
L power:GND #PWR024
U 1 1 5CAEFD3D
P 9850 1150
F 0 "#PWR024" H 9850 900 50  0001 C CNN
F 1 "GND" H 9855 977 50  0000 C CNN
F 2 "" H 9850 1150 50  0001 C CNN
F 3 "" H 9850 1150 50  0001 C CNN
	1    9850 1150
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5CAF2C23
P 9850 2200
F 0 "#PWR025" H 9850 1950 50  0001 C CNN
F 1 "GND" H 9855 2027 50  0000 C CNN
F 2 "" H 9850 2200 50  0001 C CNN
F 3 "" H 9850 2200 50  0001 C CNN
	1    9850 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5CAF7EA3
P 9850 3250
F 0 "#PWR026" H 9850 3000 50  0001 C CNN
F 1 "GND" H 9855 3077 50  0000 C CNN
F 2 "" H 9850 3250 50  0001 C CNN
F 3 "" H 9850 3250 50  0001 C CNN
	1    9850 3250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5CB0AE31
P 9850 4300
F 0 "#PWR027" H 9850 4050 50  0001 C CNN
F 1 "GND" H 9855 4127 50  0000 C CNN
F 2 "" H 9850 4300 50  0001 C CNN
F 3 "" H 9850 4300 50  0001 C CNN
	1    9850 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	10100 1300 9850 1300
Wire Wire Line
	9850 1150 9850 1300
Wire Wire Line
	10100 3400 9850 3400
Wire Wire Line
	9850 3250 9850 3400
Wire Wire Line
	10100 4450 9850 4450
Wire Wire Line
	9850 4300 9850 4450
Wire Wire Line
	10100 2350 9850 2350
Wire Wire Line
	9850 2200 9850 2350
Text Label 10100 1400 2    50   ~ 0
I2C1_SCL
Text Label 10100 1500 2    50   ~ 0
I2C1_SDA
Text Label 10100 2450 2    50   ~ 0
I2C1_SCL
Text Label 10100 2550 2    50   ~ 0
I2C1_SDA
Text Label 10100 3500 2    50   ~ 0
I2C1_SCL
Text Label 10100 3600 2    50   ~ 0
I2C1_SDA
Text Label 10100 4550 2    50   ~ 0
I2C1_SCL
Text Label 10100 4650 2    50   ~ 0
I2C1_SDA
NoConn ~ 10100 4750
NoConn ~ 10100 3700
NoConn ~ 10100 2650
NoConn ~ 10100 1600
Text Label 10100 1700 2    50   ~ 0
FR_SHUT
Text Label 10100 2750 2    50   ~ 0
FL_SHUT
Text Label 10100 3800 2    50   ~ 0
R_SHUT
Text Label 10100 4850 2    50   ~ 0
L_SHUT
Text Label 6400 4900 0    50   ~ 0
I2C1_SCL
Text Label 6400 5000 0    50   ~ 0
I2C1_SDA
Text Label 5000 4000 2    50   ~ 0
FR_SHUT
Text Label 5000 4100 2    50   ~ 0
FL_SHUT
Text Label 5000 4200 2    50   ~ 0
R_SHUT
Text Label 5000 4300 2    50   ~ 0
L_SHUT
Text Label 5000 4400 2    50   ~ 0
F_SHUT
$Comp
L power:+3.3V #PWR033
U 1 1 5CB31AB3
P 10100 5650
F 0 "#PWR033" H 10100 5500 50  0001 C CNN
F 1 "+3.3V" H 10115 5823 50  0000 C CNN
F 2 "" H 10100 5650 50  0001 C CNN
F 3 "" H 10100 5650 50  0001 C CNN
	1    10100 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 5700 10100 5700
Wire Wire Line
	10100 5700 10100 5650
$Comp
L power:GND #PWR028
U 1 1 5CB31ABB
P 9900 5650
F 0 "#PWR028" H 9900 5400 50  0001 C CNN
F 1 "GND" H 9905 5477 50  0000 C CNN
F 2 "" H 9900 5650 50  0001 C CNN
F 3 "" H 9900 5650 50  0001 C CNN
	1    9900 5650
	-1   0    0    1   
$EndComp
Wire Wire Line
	10150 5800 9900 5800
Wire Wire Line
	9900 5650 9900 5800
Text Label 10150 6000 2    50   ~ 0
I2C1_SCL
Text Label 10150 5900 2    50   ~ 0
I2C1_SDA
Text Label 10150 6100 2    50   ~ 0
F_SHUT
NoConn ~ 10150 6200
$Comp
L Device:R_Small R1
U 1 1 5CB37F61
P 4650 2600
F 0 "R1" V 4454 2600 50  0000 C CNN
F 1 "10k" V 4545 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4650 2600 50  0001 C CNN
F 3 "~" H 4650 2600 50  0001 C CNN
	1    4650 2600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5CB38F5F
P 4350 2950
F 0 "#PWR016" H 4350 2700 50  0001 C CNN
F 1 "GND" H 4355 2777 50  0000 C CNN
F 2 "" H 4350 2950 50  0001 C CNN
F 3 "" H 4350 2950 50  0001 C CNN
	1    4350 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5CB3A161
P 4650 2800
F 0 "C4" V 4800 2800 50  0000 C CNN
F 1 "2,2uF" V 4900 2800 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4688 2650 50  0001 C CNN
F 3 "~" H 4650 2800 50  0001 C CNN
	1    4650 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	4750 2600 5000 2600
Wire Wire Line
	4550 2600 4350 2600
Wire Wire Line
	4350 2600 4350 2800
Wire Wire Line
	4500 2800 4350 2800
Connection ~ 4350 2800
Wire Wire Line
	4350 2800 4350 2950
$Comp
L Device:C C3
U 1 1 5CB4AAB2
P 4650 2250
F 0 "C3" V 4400 2250 50  0000 C CNN
F 1 "2,2uF" V 4500 2250 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4688 2100 50  0001 C CNN
F 3 "~" H 4650 2250 50  0001 C CNN
	1    4650 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 2400 4950 2400
Wire Wire Line
	4950 2400 4950 2250
Wire Wire Line
	4950 2250 4800 2250
Wire Wire Line
	4500 2250 4350 2250
Wire Wire Line
	4350 2250 4350 2600
Connection ~ 4350 2600
Wire Wire Line
	5000 2800 4800 2800
NoConn ~ 5000 3500
NoConn ~ 5000 3600
$Comp
L Micromouse-rescue:Pololu_DRV8835-WK_lib U1
U 1 1 5D04D317
P 1750 4050
F 0 "U1" H 1775 4625 50  0000 C CNN
F 1 "Pololu_DRV8835" H 1775 4534 50  0000 C CNN
F 2 "WK_lib:Pololu_DRV8835_rev" H 1750 4050 50  0001 C CNN
F 3 "" H 1750 4050 50  0001 C CNN
	1    1750 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5D04F0A1
P 1250 3550
F 0 "#PWR04" H 1250 3300 50  0001 C CNN
F 1 "GND" H 1255 3377 50  0000 C CNN
F 2 "" H 1250 3550 50  0001 C CNN
F 3 "" H 1250 3550 50  0001 C CNN
	1    1250 3550
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5D04FBC6
P 2300 3550
F 0 "#PWR010" H 2300 3300 50  0001 C CNN
F 1 "GND" H 2305 3377 50  0000 C CNN
F 2 "" H 2300 3550 50  0001 C CNN
F 3 "" H 2300 3550 50  0001 C CNN
	1    2300 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	2250 3700 2300 3700
Wire Wire Line
	2300 3700 2300 3550
Wire Wire Line
	1300 3700 1250 3700
Wire Wire Line
	1250 3700 1250 3550
$Comp
L power:+3.3V #PWR011
U 1 1 5D053A9F
P 2550 3550
F 0 "#PWR011" H 2550 3400 50  0001 C CNN
F 1 "+3.3V" H 2565 3723 50  0000 C CNN
F 2 "" H 2550 3550 50  0001 C CNN
F 3 "" H 2550 3550 50  0001 C CNN
	1    2550 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3800 2550 3550
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 5D0585B6
P 800 2200
F 0 "J2" H 800 1900 50  0000 C CNN
F 1 "MOTOR_POWER_IN" H 750 2000 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 800 2200 50  0001 C CNN
F 3 "~" H 800 2200 50  0001 C CNN
	1    800  2200
	-1   0    0    1   
$EndComp
Wire Wire Line
	1150 950  1500 950 
Wire Wire Line
	1150 1300 1500 1300
$Comp
L power:GND #PWR05
U 1 1 5D066734
P 1500 1350
F 0 "#PWR05" H 1500 1100 50  0001 C CNN
F 1 "GND" H 1505 1177 50  0000 C CNN
F 2 "" H 1500 1350 50  0001 C CNN
F 3 "" H 1500 1350 50  0001 C CNN
	1    1500 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5D0687EC
P 1250 2250
F 0 "#PWR03" H 1250 2000 50  0001 C CNN
F 1 "GND" H 1255 2077 50  0000 C CNN
F 2 "" H 1250 2250 50  0001 C CNN
F 3 "" H 1250 2250 50  0001 C CNN
	1    1250 2250
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR02
U 1 1 5D068DE8
P 1250 2050
F 0 "#PWR02" H 1250 1900 50  0001 C CNN
F 1 "VCC" H 1267 2223 50  0000 C CNN
F 2 "" H 1250 2050 50  0001 C CNN
F 3 "" H 1250 2050 50  0001 C CNN
	1    1250 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2100 1250 2050
Wire Wire Line
	1250 2200 1250 2250
Wire Wire Line
	1000 2100 1250 2100
Wire Wire Line
	1000 2200 1250 2200
$Comp
L power:VCC #PWR01
U 1 1 5D078B15
P 1000 3550
F 0 "#PWR01" H 1000 3400 50  0001 C CNN
F 1 "VCC" H 1017 3723 50  0000 C CNN
F 2 "" H 1000 3550 50  0001 C CNN
F 3 "" H 1000 3550 50  0001 C CNN
	1    1000 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3800 1000 3800
Wire Wire Line
	1000 3800 1000 3550
Wire Wire Line
	1500 1350 1500 1300
Connection ~ 1500 1300
Wire Wire Line
	1500 1300 1950 1300
Text Label 1500 850  1    50   ~ 0
VMM
Wire Wire Line
	1500 850  1500 950 
Connection ~ 1500 950 
Wire Wire Line
	1500 950  1650 950 
Text Label 1300 4300 2    50   ~ 0
VMM
Wire Wire Line
	2250 4300 2550 4300
Wire Wire Line
	2550 4300 2550 3800
Connection ~ 2550 3800
Wire Wire Line
	2250 3800 2550 3800
$Comp
L power:GND #PWR06
U 1 1 5D09ADCB
P 1700 4950
F 0 "#PWR06" H 1700 4700 50  0001 C CNN
F 1 "GND" H 1705 4777 50  0000 C CNN
F 2 "" H 1700 4950 50  0001 C CNN
F 3 "" H 1700 4950 50  0001 C CNN
	1    1700 4950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5D09BC92
P 1700 6000
F 0 "#PWR07" H 1700 5750 50  0001 C CNN
F 1 "GND" H 1705 5827 50  0000 C CNN
F 2 "" H 1700 6000 50  0001 C CNN
F 3 "" H 1700 6000 50  0001 C CNN
	1    1700 6000
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR08
U 1 1 5D09E8F0
P 1900 4950
F 0 "#PWR08" H 1900 4800 50  0001 C CNN
F 1 "+3.3V" H 1915 5123 50  0000 C CNN
F 2 "" H 1900 4950 50  0001 C CNN
F 3 "" H 1900 4950 50  0001 C CNN
	1    1900 4950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5D09F268
P 1900 6000
F 0 "#PWR09" H 1900 5850 50  0001 C CNN
F 1 "+3.3V" H 1915 6173 50  0000 C CNN
F 2 "" H 1900 6000 50  0001 C CNN
F 3 "" H 1900 6000 50  0001 C CNN
	1    1900 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 5100 1700 5100
Wire Wire Line
	1700 5100 1700 4950
Wire Wire Line
	1450 5200 1900 5200
Wire Wire Line
	1900 5200 1900 4950
Wire Wire Line
	1450 6150 1700 6150
Wire Wire Line
	1700 6150 1700 6000
Wire Wire Line
	1450 6250 1900 6250
Wire Wire Line
	1900 6250 1900 6000
Text Label 1450 6350 0    50   ~ 0
R_ENC_B
Text Label 1450 6450 0    50   ~ 0
R_ENC_A
$Comp
L Connector:Conn_01x06_Female J4
U 1 1 5D0AE6A2
P 1250 6450
F 0 "J4" H 1142 5925 50  0000 C CNN
F 1 "Encoder_right" H 1142 6016 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 1250 6450 50  0001 C CNN
F 3 "~" H 1250 6450 50  0001 C CNN
	1    1250 6450
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x06_Female J3
U 1 1 5D0AF835
P 1250 5400
F 0 "J3" H 1142 4875 50  0000 C CNN
F 1 "Encoder_left" H 1142 4966 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 1250 5400 50  0001 C CNN
F 3 "~" H 1250 5400 50  0001 C CNN
	1    1250 5400
	-1   0    0    1   
$EndComp
Text Label 1450 5300 0    50   ~ 0
L_ENC_B
Text Label 1450 5400 0    50   ~ 0
L_ENC_A
Text Label 1450 5500 0    50   ~ 0
L_MOT2
Text Label 1450 5600 0    50   ~ 0
L_MOT1
Text Label 1450 6550 0    50   ~ 0
R_MOT2
Text Label 1450 6650 0    50   ~ 0
R_MOT1
Text Label 6400 3900 0    50   ~ 0
L_ENC_A
Text Label 6400 4400 0    50   ~ 0
L_ENC_B
Text Label 6400 4700 0    50   ~ 0
R_ENC_A
Text Label 6400 4800 0    50   ~ 0
R_ENC_B
Text Label 1300 4200 2    50   ~ 0
L_MOT1
Text Label 1300 4100 2    50   ~ 0
L_MOT2
Text Label 1300 4000 2    50   ~ 0
R_MOT1
Text Label 1300 3900 2    50   ~ 0
R_MOT2
Text Label 2250 3900 0    50   ~ 0
PWM_R
Text Label 2250 4000 0    50   ~ 0
DIR_R
Text Label 2250 4100 0    50   ~ 0
PWM_L
Text Label 2250 4200 0    50   ~ 0
DIR_L
Text Label 5000 4800 2    50   ~ 0
PWM_R
Text Label 5000 4600 2    50   ~ 0
PWM_L
Text Label 5000 5200 2    50   ~ 0
DIR_R
Text Label 5000 5000 2    50   ~ 0
DIR_L
$Comp
L Connector:Conn_01x08_Female J5
U 1 1 5D0C2DEC
P 3850 6800
F 0 "J5" V 3923 6730 50  0000 C CNN
F 1 "MPU6050_I2C_IMU" V 4014 6730 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 3850 6800 50  0001 C CNN
F 3 "~" H 3850 6800 50  0001 C CNN
	1    3850 6800
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR015
U 1 1 5D0C7033
P 4250 6500
F 0 "#PWR015" H 4250 6350 50  0001 C CNN
F 1 "+3.3V" V 4265 6628 50  0000 L CNN
F 2 "" H 4250 6500 50  0001 C CNN
F 3 "" H 4250 6500 50  0001 C CNN
	1    4250 6500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5D0C8390
P 4250 6350
F 0 "#PWR014" H 4250 6100 50  0001 C CNN
F 1 "GND" V 4255 6222 50  0000 R CNN
F 2 "" H 4250 6350 50  0001 C CNN
F 3 "" H 4250 6350 50  0001 C CNN
	1    4250 6350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4150 6600 4150 6500
Wire Wire Line
	4150 6500 4250 6500
Wire Wire Line
	4250 6350 4050 6350
Wire Wire Line
	4050 6350 4050 6600
Text Label 3950 6600 1    50   ~ 0
I2C1_SCL
Text Label 3850 6600 1    50   ~ 0
I2C1_SDA
NoConn ~ 3750 6600
NoConn ~ 3650 6600
NoConn ~ 3550 6600
NoConn ~ 3450 6600
Wire Wire Line
	1850 2100 1500 2100
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5D0E976C
P 1150 850
F 0 "#FLG01" H 1150 925 50  0001 C CNN
F 1 "PWR_FLAG" H 1150 1023 50  0000 C CNN
F 2 "" H 1150 850 50  0001 C CNN
F 3 "~" H 1150 850 50  0001 C CNN
	1    1150 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 850  1150 950 
NoConn ~ 5000 4500
NoConn ~ 5000 4700
NoConn ~ 5000 4900
NoConn ~ 5000 5100
NoConn ~ 5000 5300
NoConn ~ 5000 5400
NoConn ~ 5000 5500
NoConn ~ 6400 5500
NoConn ~ 6400 5400
NoConn ~ 6400 5100
NoConn ~ 6400 4600
NoConn ~ 6400 4500
NoConn ~ 6400 4300
NoConn ~ 6400 4200
NoConn ~ 6400 4100
NoConn ~ 6400 3600
NoConn ~ 6400 3500
NoConn ~ 6400 3400
NoConn ~ 6400 3300
NoConn ~ 6400 3200
NoConn ~ 6400 3100
NoConn ~ 6400 3000
NoConn ~ 6400 2900
NoConn ~ 6400 2500
NoConn ~ 6400 2400
NoConn ~ 5000 3800
Text Label 6400 2700 0    50   ~ 0
UART2_RX
Text Label 6400 2600 0    50   ~ 0
UART2_TX
$Comp
L Connector:Conn_01x04_Female J7
U 1 1 5D163181
P 8900 2050
F 0 "J7" H 8928 2026 50  0000 L CNN
F 1 "UART_female" H 8928 1935 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 8900 2050 50  0001 C CNN
F 3 "~" H 8900 2050 50  0001 C CNN
	1    8900 2050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J8
U 1 1 5D164F9A
P 8900 2600
F 0 "J8" H 8872 2482 50  0000 R CNN
F 1 "UART_male" H 8872 2573 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8900 2600 50  0001 C CNN
F 3 "~" H 8900 2600 50  0001 C CNN
	1    8900 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	8700 2400 8650 2400
Wire Wire Line
	8650 2400 8650 2250
Wire Wire Line
	8650 2250 8700 2250
Wire Wire Line
	8700 2150 8600 2150
Wire Wire Line
	8600 2150 8600 2500
Wire Wire Line
	8600 2500 8700 2500
Wire Wire Line
	8700 2600 8550 2600
Wire Wire Line
	8550 2600 8550 2050
Wire Wire Line
	8550 2050 8700 2050
Wire Wire Line
	8700 1950 8500 1950
Wire Wire Line
	8500 1950 8500 2700
Wire Wire Line
	8500 2700 8700 2700
Text Label 8350 2250 2    50   ~ 0
UART2_RX
Text Label 8350 2150 2    50   ~ 0
UART2_TX
$Comp
L power:+3.3V #PWR021
U 1 1 5D177486
P 8350 1850
F 0 "#PWR021" H 8350 1700 50  0001 C CNN
F 1 "+3.3V" H 8365 2023 50  0000 C CNN
F 2 "" H 8350 1850 50  0001 C CNN
F 3 "" H 8350 1850 50  0001 C CNN
	1    8350 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5D177CF0
P 8150 1850
F 0 "#PWR020" H 8150 1600 50  0001 C CNN
F 1 "GND" H 8155 1677 50  0000 C CNN
F 2 "" H 8150 1850 50  0001 C CNN
F 3 "" H 8150 1850 50  0001 C CNN
	1    8150 1850
	-1   0    0    1   
$EndComp
Wire Wire Line
	8600 2150 8350 2150
Connection ~ 8600 2150
Wire Wire Line
	8350 2250 8650 2250
Connection ~ 8650 2250
Wire Wire Line
	8550 2050 8150 2050
Wire Wire Line
	8150 2050 8150 1850
Connection ~ 8550 2050
Wire Wire Line
	8350 1850 8350 1950
Wire Wire Line
	8350 1950 8500 1950
Connection ~ 8500 1950
$Comp
L Switch:SW_Push SW1
U 1 1 5D18BC69
P 7300 2800
F 0 "SW1" H 7300 3085 50  0000 C CNN
F 1 "SW_Push" H 7300 2994 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 7300 3000 50  0001 C CNN
F 3 "~" H 7300 3000 50  0001 C CNN
	1    7300 2800
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR019
U 1 1 5D18D4E4
P 7650 2800
F 0 "#PWR019" H 7650 2650 50  0001 C CNN
F 1 "+3.3V" V 7665 2928 50  0000 L CNN
F 2 "" H 7650 2800 50  0001 C CNN
F 3 "" H 7650 2800 50  0001 C CNN
	1    7650 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	7650 2800 7500 2800
Wire Wire Line
	7100 2800 6400 2800
$Comp
L Device:LED D1
U 1 1 5D0A3020
P 7350 5300
F 0 "D1" H 7343 5516 50  0000 C CNN
F 1 "LED" H 7343 5425 50  0000 C CNN
F 2 "LED_THT:LED_D3.0mm" H 7350 5300 50  0001 C CNN
F 3 "~" H 7350 5300 50  0001 C CNN
	1    7350 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5D0AC535
P 6850 5300
F 0 "R2" V 6654 5300 50  0000 C CNN
F 1 "150R" V 6745 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 6850 5300 50  0001 C CNN
F 3 "~" H 6850 5300 50  0001 C CNN
	1    6850 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 5300 7200 5300
Wire Wire Line
	6400 5300 6750 5300
$Comp
L power:+3.3V #PWR034
U 1 1 5D0C6205
P 7950 5000
F 0 "#PWR034" H 7950 4850 50  0001 C CNN
F 1 "+3.3V" H 7965 5173 50  0000 C CNN
F 2 "" H 7950 5000 50  0001 C CNN
F 3 "" H 7950 5000 50  0001 C CNN
	1    7950 5000
	1    0    0    -1  
$EndComp
NoConn ~ 6400 5200
Wire Wire Line
	7500 5300 7950 5300
Wire Wire Line
	7950 5300 7950 5000
Text Label 1500 2000 1    50   ~ 0
VMM
Wire Wire Line
	1500 2000 1500 2100
Wire Wire Line
	1500 2100 1250 2100
Connection ~ 1500 2100
Connection ~ 1250 2100
$EndSCHEMATC
