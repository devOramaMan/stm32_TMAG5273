tested TMAG5273 evaluation board with nucleo stm32g474

Handle the TMAG5273 Low-Power Linear 3D Hall-Effect Sensor I2C Interface for STM32 HAL environment

In Main: 
Configure board and poll X,Y,Z, temperature, Angle and Magnitude.

 *                  - 7.5.1.3.6 I2C Read CRC - Issues - missing crc
 *                  - Utilize the readFuncPtr in:
 *                             TMAG5273_ReadTemperature
 *                             TMAG5273_ReadMagneticField
 *                             TMAG5273_ReadAngle
 *                             TMAG5273_ReadSensorData

SWO print out


See doc/ for datasheet, eval explanatory and nucleo to TMAG5273 wiring.


Test Output from a 360 deg rotation of A1:

*****************************************************************
A1 Configuration: 
DEVICE_CONFIG_1:      00110100
DEVICE_CONFIG_2:      00010000
SENSOR_CONFIG_1:      01110000
SENSOR_CONFIG_2:      00000101
X_THR_CONFIG:         00000000
Y_THR_CONFIG:         00000000
Z_THR_CONFIG:         00000000
T_CONFIG:             00000001
INT_CONFIG_1:         00000001
MAG_GAIN_CONFIG:      00000000
MAG_OFFSET_CONFIG_1:  00000000
MAG_OFFSET_CONFIG_2:  00000000
DEVICE_STATUS:        00001000
*****************************************************************
*****************************************************************
A2 Configuration: 
DEVICE_CONFIG_1:      00110100
DEVICE_CONFIG_2:      00010000
SENSOR_CONFIG_1:      01110000
SENSOR_CONFIG_2:      00000101
X_THR_CONFIG:         00000000
Y_THR_CONFIG:         00000000
Z_THR_CONFIG:         00000000
T_CONFIG:             00000001
INT_CONFIG_1:         00000001
MAG_GAIN_CONFIG:      00000000
MAG_OFFSET_CONFIG_1:  00000000
MAG_OFFSET_CONFIG_2:  00000000
DEVICE_STATUS:        00001000
*****************************************************************

A1 X:   78.6938, Y:  -78.8892, Z:    2.8149, temperature   22.6373, Status 0x71, Angle:   314.75, magnitude:      178	             A2 X:   -0.2516, Y:   -0.4059, Z:    0.0731, temperature   24.6339, status 0x71, Angle:   244.50, magnitude:        0
A1 X:   78.6890, Y:  -78.8843, Z:    2.7832, temperature   22.5874, Status 0x91, Angle:   314.75, magnitude:      178	             A2 X:   -0.2192, Y:   -0.3004, Z:    0.0406, temperature   24.5840, status 0x91, Angle:   237.50, magnitude:        0
A1 X:   78.6938, Y:  -78.8892, Z:    2.7881, temperature   22.6040, Status 0xB1, Angle:   314.75, magnitude:      178	             A2 X:   -0.2029, Y:   -0.3653, Z:    0.0568, temperature   24.5840, status 0xB1, Angle:   237.50, magnitude:        0
A1 X:   78.6865, Y:  -78.8818, Z:    2.8174, temperature   22.5707, Status 0xD1, Angle:   314.75, magnitude:      178	             A2 X:   -0.2273, Y:   -0.3572, Z:    0.0568, temperature   24.5674, status 0xD1, Angle:   237.50, magnitude:        0
A1 X:   78.6963, Y:  -78.8916, Z:    1.7871, temperature   22.6040, Status 0xF1, Angle:   314.75, magnitude:      178	             A2 X:   -0.2354, Y:   -0.3572, Z:    0.0325, temperature   24.6173, status 0xF1, Angle:   237.50, magnitude:        0
A1 X:   78.6987, Y:  -78.8940, Z:    1.6870, temperature   22.6705, Status 0x11, Angle:   314.75, magnitude:      178	             A2 X:   -0.1948, Y:   -0.2679, Z:    0.0649, temperature   24.5840, status 0x11, Angle:   237.50, magnitude:        0
A1 X:   78.6914, Y:  -77.0752, Z:    1.2329, temperature   22.5874, Status 0x31, Angle:   315.75, magnitude:      176	             A2 X:   -0.1867, Y:   -0.3166, Z:    0.0568, temperature   24.7005, status 0x31, Angle:   237.50, magnitude:        0
A1 X:   78.7012, Y:  -49.0356, Z:   -1.0986, temperature   22.6705, Status 0x51, Angle:   328.25, magnitude:      148	             A2 X:   -0.2354, Y:   -0.1705, Z:    0.0568, temperature   24.5674, status 0x51, Angle:   223.50, magnitude:        0
A1 X:   78.6987, Y:  -31.2036, Z:   -1.4697, temperature   22.6373, Status 0x71, Angle:   338.50, magnitude:      135	             A2 X:   -0.2679, Y:   -0.2111, Z:    0.0406, temperature   24.6839, status 0x71, Angle:   212.50, magnitude:        0
A1 X:   78.6963, Y:    8.3252, Z:   -3.2397, temperature   22.6206, Status 0x91, Angle:     6.00, magnitude:      126	             A2 X:   -0.1786, Y:   -0.1055, Z:    0.0325, temperature   24.6672, status 0x91, Angle:   205.50, magnitude:        0
A1 X:   78.6890, Y:   31.2183, Z:   -4.3896, temperature   22.5374, Status 0xB1, Angle:    21.50, magnitude:      135	             A2 X:   -0.2029, Y:   -0.1461, Z:    0.0325, temperature   24.6839, status 0xB1, Angle:   223.50, magnitude:        0
A1 X:   78.6865, Y:   51.6650, Z:   -4.7852, temperature   22.5374, Status 0xD1, Angle:    33.25, magnitude:      150	             A2 X:   -0.1218, Y:   -0.0893, Z:    0.0000, temperature   24.6839, status 0xD1, Angle:   223.50, magnitude:        0
A1 X:   78.6987, Y:   71.5405, Z:   -5.8325, temperature   22.6539, Status 0xF1, Angle:    42.25, magnitude:      170	             A2 X:   -0.0568, Y:   -0.0974, Z:   -0.0244, temperature   24.6839, status 0xF1, Angle:   223.50, magnitude:        0
A1 X:   78.6963, Y:   79.9609, Z:   -6.8604, temperature   22.6705, Status 0x11, Angle:    45.50, magnitude:      179	             A2 X:   -0.0406, Y:   -0.1542, Z:   -0.0244, temperature   24.6506, status 0x11, Angle:   245.00, magnitude:        0
A1 X:   78.6914, Y:   79.9609, Z:   -7.0752, temperature   22.5874, Status 0x31, Angle:    45.50, magnitude:      179	             A2 X:   -0.0244, Y:   -0.0568, Z:   -0.0487, temperature   24.7005, status 0x31, Angle:   223.50, magnitude:        0
A1 X:   78.7036, Y:   79.9609, Z:   -7.0581, temperature   22.6539, Status 0x51, Angle:    45.50, magnitude:      179	             A2 X:   -0.0244, Y:   -0.0812, Z:   -0.0406, temperature   24.7171, status 0x51, Angle:   223.50, magnitude:        0
A1 X:   78.6914, Y:   79.9609, Z:   -7.0166, temperature   22.5374, Status 0x71, Angle:    45.50, magnitude:      179	             A2 X:   -0.1299, Y:   -0.0893, Z:   -0.0162, temperature   24.7671, status 0x71, Angle:   223.50, magnitude:        0
A1 X:   78.6914, Y:   79.9609, Z:   -9.1357, temperature   22.5541, Status 0x91, Angle:    45.50, magnitude:      179	             A2 X:    0.0325, Y:   -0.0731, Z:   -0.0812, temperature   24.6339, status 0x91, Angle:   260.25, magnitude:        0
A1 X:   52.2046, Y:   79.9609, Z:  -10.2222, temperature   22.5874, Status 0xB1, Angle:    56.75, magnitude:      152	             A2 X:    0.0244, Y:   -0.1948, Z:   -0.1136, temperature   24.7005, status 0xB1, Angle:   267.50, magnitude:        0
A1 X:   30.0488, Y:   79.9609, Z:  -10.4468, temperature   22.6539, Status 0xD1, Angle:    69.50, magnitude:      136	             A2 X:    0.0893, Y:   -0.1786, Z:   -0.1461, temperature   24.6339, status 0xD1, Angle:   267.50, magnitude:        0
A1 X:    7.2925, Y:   79.9609, Z:  -10.5542, temperature   22.7371, Status 0xF1, Angle:    84.75, magnitude:      128	             A2 X:    0.0649, Y:   -0.2111, Z:   -0.1786, temperature   24.6672, status 0xF1, Angle:   267.50, magnitude:        0
A1 X:  -16.3818, Y:   79.9609, Z:  -10.5811, temperature   22.7205, Status 0x11, Angle:   101.75, magnitude:      130	             A2 X:    0.0893, Y:   -0.1542, Z:   -0.1948, temperature   24.6506, status 0x11, Angle:   267.50, magnitude:        0
A1 X:  -32.3950, Y:   79.9609, Z:  -10.3638, temperature   22.7537, Status 0x31, Angle:   112.00, magnitude:      138	             A2 X:    0.1461, Y:   -0.1867, Z:   -0.1867, temperature   24.6007, status 0x31, Angle:   295.00, magnitude:        0
A1 X:  -43.2886, Y:   79.9609, Z:  -10.5908, temperature   22.7205, Status 0x51, Angle:   118.25, magnitude:      145	             A2 X:    0.1380, Y:   -0.2273, Z:   -0.2111, temperature   24.6506, status 0x51, Angle:   295.00, magnitude:        0
A1 X:  -64.0576, Y:   79.9609, Z:  -10.3174, temperature   22.7371, Status 0x71, Angle:   128.50, magnitude:      164	             A2 X:    0.1299, Y:   -0.2841, Z:   -0.2273, temperature   24.6506, status 0x71, Angle:   288.50, magnitude:        0
A1 X:  -75.8252, Y:   79.9609, Z:  -10.8716, temperature   22.7537, Status 0x91, Angle:   133.50, magnitude:      176	             A2 X:    0.1705, Y:   -0.2679, Z:   -0.2273, temperature   24.5507, status 0x91, Angle:   288.50, magnitude:        0
A1 X:  -79.9609, Y:   79.9609, Z:  -10.9277, temperature   22.6206, Status 0xB1, Angle:   135.00, magnitude:      181	             A2 X:    0.1461, Y:   -0.2516, Z:   -0.2354, temperature   24.7171, status 0xB1, Angle:   295.00, magnitude:        0
A1 X:  -79.9609, Y:   79.9609, Z:  -11.1133, temperature   22.7205, Status 0xD1, Angle:   135.00, magnitude:      181	             A2 X:    0.1380, Y:   -0.3247, Z:   -0.2273, temperature   24.6173, status 0xD1, Angle:   288.50, magnitude:        0
A1 X:  -79.9609, Y:   79.9609, Z:  -11.0449, temperature   22.6373, Status 0xF1, Angle:   135.00, magnitude:      181	             A2 X:    0.1055, Y:   -0.2760, Z:   -0.2273, temperature   24.5674, status 0xF1, Angle:   267.50, magnitude:        0
A1 X:  -79.9609, Y:   79.9609, Z:  -11.0059, temperature   22.6872, Status 0x11, Angle:   135.00, magnitude:      181	             A2 X:    0.1705, Y:   -0.3085, Z:   -0.2354, temperature   24.6506, status 0x11, Angle:   288.50, magnitude:        0
A1 X:  -79.9609, Y:   79.9609, Z:   -7.9517, temperature   22.6705, Status 0x31, Angle:   135.00, magnitude:      181	             A2 X:    0.0568, Y:   -0.4465, Z:   -0.2111, temperature   24.7005, status 0x31, Angle:   267.50, magnitude:        0
A1 X:  -79.9609, Y:   40.6543, Z:   -5.7690, temperature   22.6705, Status 0x51, Angle:   153.00, magnitude:      143	             A2 X:    0.1624, Y:   -0.4546, Z:   -0.2111, temperature   24.6839, status 0x51, Angle:   285.00, magnitude:        0
A1 X:  -79.9609, Y:   12.3120, Z:   -4.6704, temperature   22.6373, Status 0x71, Angle:   171.25, magnitude:      129	             A2 X:    0.1218, Y:   -0.5195, Z:   -0.1948, temperature   24.5341, status 0x71, Angle:   267.50, magnitude:        0
A1 X:  -79.9609, Y:  -17.6660, Z:   -3.6548, temperature   22.6705, Status 0x91, Angle:   192.50, magnitude:      131	             A2 X:    0.0649, Y:   -0.5520, Z:   -0.2192, temperature   24.6506, status 0x91, Angle:   269.25, magnitude:        0
A1 X:  -79.9609, Y:  -49.3018, Z:   -3.2935, temperature   22.6872, Status 0xB1, Angle:   211.50, magnitude:      150	             A2 X:    0.1055, Y:   -0.5601, Z:   -0.2111, temperature   24.5341, status 0xB1, Angle:   269.25, magnitude:        0
A1 X:  -79.9609, Y:  -74.5605, Z:   -2.0410, temperature   22.6872, Status 0xD1, Angle:   223.00, magnitude:      174	             A2 X:    0.1055, Y:   -0.5845, Z:   -0.1786, temperature   24.6339, status 0xD1, Angle:   269.25, magnitude:        0
A1 X:  -79.9609, Y:  -78.8989, Z:   -0.7056, temperature   22.7371, Status 0xF1, Angle:   224.50, magnitude:      179	             A2 X:    0.0162, Y:   -0.5845, Z:   -0.1705, temperature   24.7171, status 0xF1, Angle:   269.25, magnitude:        0
A1 X:  -79.9609, Y:  -78.9038, Z:    0.2417, temperature   22.7371, Status 0x11, Angle:   224.50, magnitude:      179	             A2 X:    0.0081, Y:   -0.5439, Z:   -0.1218, temperature   24.7171, status 0x11, Angle:   269.25, magnitude:        0
A1 X:  -62.9736, Y:  -78.8818, Z:   -0.3735, temperature   22.6206, Status 0x31, Angle:   231.50, magnitude:      161	             A2 X:   -0.0568, Y:   -0.5764, Z:   -0.0974, temperature   24.5341, status 0x31, Angle:   258.75, magnitude:        0
A1 X:  -21.3550, Y:  -78.8965, Z:    1.4111, temperature   22.6539, Status 0x51, Angle:   254.75, magnitude:      130	             A2 X:   -0.0568, Y:   -0.4952, Z:   -0.0731, temperature   24.6672, status 0x51, Angle:   255.00, magnitude:        0
A1 X:  -21.3477, Y:  -78.9111, Z:    1.2646, temperature   22.8203, Status 0x71, Angle:   254.75, magnitude:      131	             A2 X:   -0.1055, Y:   -0.5276, Z:   -0.0162, temperature   24.6506, status 0x71, Angle:   258.75, magnitude:        0
A1 X:   44.1479, Y:  -78.8989, Z:    2.7734, temperature   22.7205, Status 0x91, Angle:   299.25, magnitude:      144	             A2 X:   -0.0812, Y:   -0.5195, Z:    0.0162, temperature   24.6506, status 0x91, Angle:   255.00, magnitude:        0
A1 X:   76.2280, Y:  -78.8940, Z:    2.2119, temperature   22.6705, Status 0xB1, Angle:   314.00, magnitude:      175	             A2 X:   -0.1542, Y:   -0.4465, Z:    0.0244, temperature   24.7504, status 0xB1, Angle:   244.50, magnitude:        0
A1 X:   78.7012, Y:  -78.8965, Z:    2.1582, temperature   22.6872, Status 0xD1, Angle:   314.75, magnitude:      178	             A2 X:   -0.1055, Y:   -0.4059, Z:    0.0244, temperature   24.6672, status 0xD1, Angle:   255.00, magnitude:        0
A1 X:   78.7012, Y:  -78.8965, Z:    1.1450, temperature   22.7870, Status 0xF1, Angle:   314.75, magnitude:      178	             A2 X:   -0.1299, Y:   -0.3815, Z:    0.0162, temperature   24.7005, status 0xF1, Angle:   251.50, magnitude:        0
A1 X:   78.7012, Y:  -78.8965, Z:    0.7935, temperature   22.7038, Status 0x11, Angle:   314.75, magnitude:      178	             A2 X:   -0.2273, Y:   -0.3491, Z:    0.0406, temperature   24.7504, status 0x11, Angle:   237.50, magnitude:        0
A1 X:   78.7085, Y:  -76.7041, Z:    0.6982, temperature   22.7704, Status 0x31, Angle:   315.75, magnitude:      175	             A2 X:   -0.1299, Y:   -0.3004, Z:    0.0812, temperature   24.6339, status 0x31, Angle:   251.50, magnitude:        0
A1 X:   78.7158, Y:  -76.6333, Z:    0.7544, temperature   22.7870, Status 0x51, Angle:   316.00, magnitude:      175	             A2 X:   -0.1542, Y:   -0.2922, Z:    0.0244, temperature   24.6672, status 0x51, Angle:   237.50, magnitude:        0
A1 X:   78.7012, Y:  -76.7651, Z:    0.7178, temperature   22.7038, Status 0x71, Angle:   315.75, magnitude:      175	             A2 X:   -0.0487, Y:   -0.2922, Z:    0.0568, temperature   24.5507, status 0x71, Angle:   251.50, magnitude:        0
A1 X:   78.7012, Y:  -76.7358, Z:    0.7202, temperature   22.7038, Status 0x91, Angle:   315.75, magnitude:      175	             A2 X:   -0.0974, Y:   -0.3085, Z:    0.0487, temperature   24.7005, status 0x91, Angle:   251.50, magnitude:        0
A1 X:   78.7158, Y:  -67.3828, Z:   -0.2856, temperature   22.8702, Status 0xB1, Angle:   319.50, magnitude:      165	             A2 X:   -0.0244, Y:   -0.3247, Z:    0.0325, temperature   24.6672, status 0xB1, Angle:   251.50, magnitude:        0
A1 X:   78.7036, Y:  -66.9434, Z:   -0.0562, temperature   22.7038, Status 0xD1, Angle:   319.50, magnitude:      165	             A2 X:   -0.3896, Y:   -0.1624, Z:    0.0974, temperature   24.7338, status 0xD1, Angle:   212.50, magnitude:        0
A1 X:   78.7061, Y:  -66.9702, Z:   -0.0928, temperature   22.7704, Status 0xF1, Angle:   319.50, magnitude:      165	             A2 X:   -0.8767, Y:    0.2435, Z:    0.0162, temperature   24.5840, status 0xF1, Angle:   172.00, magnitude:        0
A1 X:   78.7134, Y:  -67.1021, Z:   -0.0732, temperature   22.8369, Status 0x11, Angle:   319.50, magnitude:      165	             A2 X:   -0.9011, Y:    0.3896, Z:    0.0162, temperature   24.6506, status 0x11, Angle:   156.75, magnitude:        0
A1 X:   78.7085, Y:  -67.0532, Z:   -0.0903, temperature   22.7704, Status 0x31, Angle:   319.50, magnitude:      165	             A2 X:   -0.9417, Y:    0.2273, Z:    0.0081, temperature   24.6672, status 0x31, Angle:   172.25, magnitude:        0
A1 X:   78.7109, Y:  -67.0996, Z:   -0.0903, temperature   22.8536, Status 0x51, Angle:   319.50, magnitude:      165	             A2 X:   -0.8605, Y:    0.1948, Z:   -0.0081, temperature   24.5341, status 0x51, Angle:   172.00, magnitude:        0
A1 X:   78.7085, Y:  -67.1313, Z:   -0.0903, temperature   22.7371, Status 0x71, Angle:   319.50, magnitude:      165	             A2 X:   -0.9660, Y:    0.2922, Z:    0.0162, temperature   24.6672, status 0x71, Angle:   165.00, magnitude:        0
A1 X:   78.7134, Y:  -67.0850, Z:   -0.0977, temperature   22.8037, Status 0x91, Angle:   319.50, magnitude:      165	             A2 X:   -0.9660, Y:    0.2922, Z:   -0.0162, temperature   24.6007, status 0x91, Angle:   165.00, magnitude:        0
A1 X:   78.7134, Y:  -66.9995, Z:   -0.0903, temperature   22.8037, Status 0xB1, Angle:   319.50, magnitude:      165	             A2 X:   -0.9498, Y:    0.3328, Z:   -0.0244, temperature   24.5341, status 0xB1, Angle:   165.00, magnitude:        0