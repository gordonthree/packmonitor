#pragma once

#define PM_REGISTER_HIGHCURRENTLIMIT    0x21 // read / write high current cut off register
#define PM_REGISTER_HIGHTEMPLIMIT       0x22 // read / write high temperature cut off register
#define PM_REGISTER_LOWTEMPLIMIT        0x23 // read / write low temperature cut off register
#define PM_REGISTER_HIGHVOLTLIMIT       0x24 // read / write high voltage cut off register
#define PM_REGISTER_LOWVOLTLIMIT        0x25 // read / writte low voltage cut off register
#define PM_REGISTER_CONFIG0BYTE         0x26 // read / write config0 register
#define PM_REGISTER_CONFIG1BYTE         0x27 // read / write config1 register
#define PM_REGISTER_CONFIG2BYTE         0x28 // read / write config2 register
#define PM_REGISTER_CURRENTMVA          0x29 // read / write current sensor mva value in millivolts
#define PM_REGISTER_VPACKDIVISOR        0X2A // read / write pack voltage sense divisor
#define PM_REGISTER_VBUSDIVISOR         0x2B // read / write bus voltage sense divisor
#define PM_REGISTER_STATUS0BYTE         0x2C // read status0 register byte
#define PM_REGISTER_STATUS1BYTE         0x2D // read status1 register byte
#define PM_REGISTER_THERMDIVISOR        0x2E // read / write thermistor scaling factor
#define PM_REGISTER_VBUSVOLTS           0x2F // read / write bus voltage

#define PM_REGISTER_CLEARCOLCNTR        0x30 // write only clear coulomb counter
#define PM_REGISTER_CLEARAMPSCNTRS      0x31 // write only clear amperage counters
#define PM_REGISTER_READCOLCNTR         0x32 // read coulomb counter
#define PM_REGISTER_READLOADAMPS        0x33 // display current load amperage
#define PM_REGISTER_TOTALAMPSIN         0x34 // total amps in
#define PM_REGISTER_TOTALAMPSOUT        0x35 // total amps out
#define PM_REGISTER_LIFEAMPSIN          0x36 // display lifetime total charge amps
#define PM_REGISTER_LIFEAMPSOUT         0x37 // display lifetime total discharge amps
#define PM_REGISTER_READLOADAMPSHI      0x38 // read peak load amperage
#define PM_REGISTER_READBUSVOLTS        0x39 // read bus voltage right now
#define PM_REGISTER_READPACKVOLTS       0x3A // read pack voltage right now
#define PM_REGISTER_READLOWVOLTS        0x3B // read low pack voltage record
#define PM_REGISTER_READHIVOLTS         0x3C // read high pack voltage record
#define PM_REGISTER_READBUSVOLTSLO      0x3D // read low bus voltage 
#define PM_REGISTER_READBUSVOLTSHI      0x3E // read high bus voltage
#define PM_REGISTER_CLEARVOLTMEM        0x3F // clear voltage record memory

#define PM_REGISTER_CLEARTEMPS          0x40 // clear temperature record memory
#define PM_REGISTER_READDEGCT0          0x41 // read t0 degrees c
#define PM_REGISTER_READDEGCT1          0x42 // read t1 degrees c
#define PM_REGISTER_READDEGCT2          0x43 // read t2 degrees c
#define PM_REGISTER_READT0LOW           0x44 // read t0 low temp record
#define PM_REGISTER_READT1LOW           0x45 // read t1 low temp record
#define PM_REGISTER_READT2LOW           0x46 // read t2 low temp record
#define PM_REGISTER_READT0HIGH          0x47 // read t0 high temp record
#define PM_REGISTER_READT1HIGH          0x48 // read t1 high temp record
#define PM_REGISTER_READT2HIGH          0x49 // read t2 high temp record

#define PM_REGISTER_CLEARDISCHIST       0x50 // clear disconnect history
#define PM_REGISTER_LASTDISCREASON      0x51 // last disconnect reason code
#define PM_REGISTER_TOTOVRCURDISC       0x52 // total over current disconnects
#define PM_REGISTER_TOTUNDRVLTDISC      0x53 // total under voltage disconnects
#define PM_REGISTER_TOTOVRVLTDISC       0x54 // total over voltage disconnects
#define PM_REGISTER_TOTLOWRTEMPDISC     0x55 // total low temp disconnects
#define PM_REGISTER_TOTHITEMPDISC       0x56 // total high temp disconnects

#define PM_REGISTER_SETEPOCHTIME        0x60 // set epoch time
#define PM_REGISTER_FIRSTINITTIME       0x61 // timestamp of last eeprom initilization
#define PM_REGISTER_CURRENTTIME         0x62 // read current epoch time
#define PM_REGISTER_TIMESYNC            0x63 // elapsed time since last sync
#define PM_REGISTER_UPTIME              0x64 // elapsed time since last power-on reset

#define PM_CONFIG0_DISABLEPROTS         0x07 // set to disable all protections, monitor pack only
#define PM_CONFIG0_ENAOVRCURPROT        0x06 // set to enable over-current protection
#define PM_CONFIG0_ENAOVRTMPPROT        0x05 // set to enable over-temp protection
#define PM_CONFIG0_ENAUNDTMPPROT        0x04 // set to enable under-temp protection
#define PM_CONFIG0_EMAUNDVLTPROT        0x03 // set to enable under-voltage protection
#define PM_CONFIG0_ENAOVRVLTPROT        0x02 // set to enable over-voltage protection
#define PM_CONFIG0_ENASTATUSLEDS        0x00 // set to enable status leds if present

#define PM_CONFIG1_REFALWAYSON          0x07 // set to keep ADC voltage reference powered up all the time, clear vref as needed
#define PM_CONFIG1_AUTOVBUS             0x06 // set to enable bus voltage autodetect
#define PM_CONFIG1_REFSEL2              0x02 // three bits to select internal voltage reference level
#define PM_CONFIG1_REFSEL1              0x01 // three bits to select internal voltage reference level
#define PM_CONFIG1_REFSEL0              0x00 // three bits to select internal voltage reference level

#define PM_CONFIG2_FRAMSAVE3            0x02 // four bits to set save to fram interval bit3
#define PM_CONFIG2_FRAMSAVE2            0x02 // four bits to set save to fram interval bit2
#define PM_CONFIG2_FRAMSAVE1            0x01 // four bits to set save to fram interval bit1
#define PM_CONFIG2_FRAMSAVE0            0x00 // four bits to set save to fram interval bit0

#define PM_STATUS0_CONFIGSET            0x07 // set when config0 contains vaild configuration
#define PM_STATUS0_TIMESET              0x06 // set when system clock has been set
#define PM_STATUS0_WARNTEMP             0x05 // set when temperature within 3 degrees of either limit
#define PM_STATUS0_WARNCURRENT          0x04 // set when current within 1 amp of limit
#define PM_STATUS0_WARNVOLTAGE          0x03 // set when voltage within 0.25v of limits
#define PM_STATUS0_RANGETSNS            0x02 // set when temperature sensor value is out of range
#define PM_STATUS0_RANGEISNS            0x01 // set when current sensor value is out of range
#define PM_STATUS0_RANGEVSNS            0x00 // set when voltage sensor value is out of range

#define PM_STATUS1_RANGEVBUS            0x00 // set when bus voltage is out of normal range
#define PM_STATUS1_ALARMVPACK           0x01 // set when pack voltage alarm is present
#define PM_STATUS1_ALARMOVRCUR          0x02 // set when over current alarm is present
#define PM_STATUS1_ALARMLOWT            0x03 // set when pack low-temp alarm is present
#define PM_STATUS1_ALARMHIGHT           0x04 // set when pack high-temp alarm is present

/**
* The NTC table has 2049 interpolation points.
* Unit:0.01 °C
*
*/
int NTC_table[2049] = {
  -8455, -7957, -7459, -7117, -6854, -6638, 
  -6455, -6296, -6155, -6027, -5911, -5805, 
  -5706, -5614, -5528, -5447, -5370, -5298, 
  -5229, -5163, -5100, -5040, -4982, -4926, 
  -4872, -4821, -4771, -4722, -4675, -4630, 
  -4586, -4543, -4501, -4460, -4421, -4382, 
  -4344, -4307, -4271, -4236, -4202, -4168, 
  -4135, -4103, -4071, -4040, -4009, -3979, 
  -3950, -3921, -3893, -3865, -3837, -3810, 
  -3783, -3757, -3731, -3706, -3681, -3656, 
  -3632, -3608, -3584, -3561, -3538, -3515, 
  -3492, -3470, -3448, -3427, -3405, -3384, 
  -3363, -3343, -3322, -3302, -3282, -3262, 
  -3243, -3223, -3204, -3185, -3167, -3148, 
  -3130, -3111, -3093, -3076, -3058, -3040, 
  -3023, -3006, -2989, -2972, -2955, -2939, 
  -2922, -2906, -2890, -2874, -2858, -2842, 
  -2826, -2811, -2795, -2780, -2765, -2750, 
  -2735, -2720, -2705, -2691, -2676, -2662, 
  -2648, -2633, -2619, -2605, -2591, -2578, 
  -2564, -2550, -2537, -2523, -2510, -2497, 
  -2483, -2470, -2457, -2444, -2431, -2419, 
  -2406, -2393, -2381, -2368, -2356, -2343, 
  -2331, -2319, -2307, -2295, -2283, -2271, 
  -2259, -2247, -2235, -2224, -2212, -2201, 
  -2189, -2178, -2166, -2155, -2144, -2133, 
  -2121, -2110, -2099, -2088, -2077, -2066, 
  -2056, -2045, -2034, -2023, -2013, -2002, 
  -1992, -1981, -1971, -1960, -1950, -1940, 
  -1930, -1919, -1909, -1899, -1889, -1879, 
  -1869, -1859, -1849, -1839, -1830, -1820, 
  -1810, -1800, -1791, -1781, -1772, -1762, 
  -1752, -1743, -1734, -1724, -1715, -1705, 
  -1696, -1687, -1678, -1669, -1659, -1650, 
  -1641, -1632, -1623, -1614, -1605, -1596, 
  -1587, -1579, -1570, -1561, -1552, -1543, 
  -1535, -1526, -1517, -1509, -1500, -1492, 
  -1483, -1475, -1466, -1458, -1449, -1441, 
  -1432, -1424, -1416, -1408, -1399, -1391, 
  -1383, -1375, -1366, -1358, -1350, -1342, 
  -1334, -1326, -1318, -1310, -1302, -1294, 
  -1286, -1278, -1270, -1263, -1255, -1247, 
  -1239, -1231, -1224, -1216, -1208, -1201, 
  -1193, -1185, -1178, -1170, -1162, -1155, 
  -1147, -1140, -1132, -1125, -1117, -1110, 
  -1103, -1095, -1088, -1080, -1073, -1066, 
  -1058, -1051, -1044, -1037, -1029, -1022, 
  -1015, -1008, -1001, -993, -986, -979, -972, 
  -965, -958, -951, -944, -937, -930, -923, 
  -916, -909, -902, -895, -888, -881, -875, 
  -868, -861, -854, -847, -840, -834, -827, 
  -820, -813, -807, -800, -793, -787, -780, 
  -773, -767, -760, -753, -747, -740, -734, 
  -727, -720, -714, -707, -701, -694, -688, 
  -681, -675, -669, -662, -656, -649, -643, 
  -636, -630, -624, -617, -611, -605, -598, 
  -592, -586, -580, -573, -567, -561, -555, 
  -548, -542, -536, -530, -524, -517, -511, 
  -505, -499, -493, -487, -481, -475, -468, 
  -462, -456, -450, -444, -438, -432, -426, 
  -420, -414, -408, -402, -396, -390, -384, 
  -378, -373, -367, -361, -355, -349, -343, 
  -337, -331, -325, -320, -314, -308, -302, 
  -296, -291, -285, -279, -273, -267, -262, 
  -256, -250, -244, -239, -233, -227, -222, 
  -216, -210, -205, -199, -193, -188, -182, 
  -176, -171, -165, -159, -154, -148, -143, 
  -137, -132, -126, -120, -115, -109, -104, 
  -98, -93, -87, -82, -76, -71, -65, -60, -54, 
  -49, -43, -38, -32, -27, -22, -16, -11, -5, 
  0, 5, 11, 16, 22, 27, 32, 38, 43, 48, 54, 
  59, 64, 70, 75, 80, 86, 91, 96, 102, 107, 
  112, 117, 123, 128, 133, 138, 144, 149, 154, 
  159, 165, 170, 175, 180, 185, 191, 196, 201, 
  206, 211, 216, 222, 227, 232, 237, 242, 247, 
  252, 257, 263, 268, 273, 278, 283, 288, 293, 
  298, 303, 308, 313, 319, 324, 329, 334, 339, 
  344, 349, 354, 359, 364, 369, 374, 379, 384, 
  389, 394, 399, 404, 409, 414, 419, 424, 429, 
  434, 439, 444, 448, 453, 458, 463, 468, 473, 
  478, 483, 488, 493, 498, 503, 507, 512, 517, 
  522, 527, 532, 537, 542, 546, 551, 556, 561, 
  566, 571, 576, 580, 585, 590, 595, 600, 604, 
  609, 614, 619, 624, 629, 633, 638, 643, 648, 
  652, 657, 662, 667, 672, 676, 681, 686, 691, 
  695, 700, 705, 710, 714, 719, 724, 729, 733, 
  738, 743, 747, 752, 757, 762, 766, 771, 776, 
  780, 785, 790, 794, 799, 804, 808, 813, 818, 
  822, 827, 832, 836, 841, 846, 850, 855, 860, 
  864, 869, 874, 878, 883, 888, 892, 897, 901, 
  906, 911, 915, 920, 925, 929, 934, 938, 943, 
  948, 952, 957, 961, 966, 971, 975, 980, 984, 
  989, 993, 998, 1003, 1007, 1012, 1016, 1021, 
  1025, 1030, 1035, 1039, 1044, 1048, 1053, 
  1057, 1062, 1066, 1071, 1075, 1080, 1084, 
  1089, 1094, 1098, 1103, 1107, 1112, 1116, 
  1121, 1125, 1130, 1134, 1139, 1143, 1148, 
  1152, 1157, 1161, 1166, 1170, 1175, 1179, 
  1184, 1188, 1193, 1197, 1202, 1206, 1211, 
  1215, 1220, 1224, 1228, 1233, 1237, 1242, 
  1246, 1251, 1255, 1260, 1264, 1269, 1273, 
  1278, 1282, 1286, 1291, 1295, 1300, 1304, 
  1309, 1313, 1318, 1322, 1326, 1331, 1335, 
  1340, 1344, 1349, 1353, 1358, 1362, 1366, 
  1371, 1375, 1380, 1384, 1389, 1393, 1397, 
  1402, 1406, 1411, 1415, 1419, 1424, 1428, 
  1433, 1437, 1441, 1446, 1450, 1455, 1459, 
  1463, 1468, 1472, 1477, 1481, 1485, 1490, 
  1494, 1499, 1503, 1507, 1512, 1516, 1521, 
  1525, 1529, 1534, 1538, 1542, 1547, 1551, 
  1556, 1560, 1564, 1569, 1573, 1578, 1582, 
  1586, 1591, 1595, 1599, 1604, 1608, 1612, 
  1617, 1621, 1626, 1630, 1634, 1639, 1643, 
  1647, 1652, 1656, 1661, 1665, 1669, 1674, 
  1678, 1682, 1687, 1691, 1695, 1700, 1704, 
  1708, 1713, 1717, 1722, 1726, 1730, 1735, 
  1739, 1743, 1748, 1752, 1756, 1761, 1765, 
  1769, 1774, 1778, 1782, 1787, 1791, 1795, 
  1800, 1804, 1808, 1813, 1817, 1822, 1826, 
  1830, 1835, 1839, 1843, 1848, 1852, 1856, 
  1861, 1865, 1869, 1874, 1878, 1882, 1887, 
  1891, 1895, 1900, 1904, 1908, 1913, 1917, 
  1921, 1926, 1930, 1934, 1939, 1943, 1947, 
  1952, 1956, 1960, 1965, 1969, 1973, 1978, 
  1982, 1986, 1991, 1995, 1999, 2004, 2008, 
  2012, 2017, 2021, 2025, 2030, 2034, 2038, 
  2043, 2047, 2051, 2056, 2060, 2064, 2069, 
  2073, 2078, 2082, 2086, 2091, 2095, 2099, 
  2104, 2108, 2112, 2117, 2121, 2125, 2130, 
  2134, 2138, 2143, 2147, 2151, 2156, 2160, 
  2164, 2169, 2173, 2177, 2182, 2186, 2190, 
  2195, 2199, 2203, 2208, 2212, 2217, 2221, 
  2225, 2230, 2234, 2238, 2243, 2247, 2251, 
  2256, 2260, 2264, 2269, 2273, 2278, 2282, 
  2286, 2291, 2295, 2299, 2304, 2308, 2312, 
  2317, 2321, 2325, 2330, 2334, 2339, 2343, 
  2347, 2352, 2356, 2360, 2365, 2369, 2374, 
  2378, 2382, 2387, 2391, 2395, 2400, 2404, 
  2409, 2413, 2417, 2422, 2426, 2430, 2435, 
  2439, 2444, 2448, 2452, 2457, 2461, 2466, 
  2470, 2474, 2479, 2483, 2488, 2492, 2496, 
  2501, 2505, 2510, 2514, 2518, 2523, 2527, 
  2532, 2536, 2540, 2545, 2549, 2554, 2558, 
  2562, 2567, 2571, 2576, 2580, 2585, 2589, 
  2593, 2598, 2602, 2607, 2611, 2616, 2620, 
  2624, 2629, 2633, 2638, 2642, 2647, 2651, 
  2655, 2660, 2664, 2669, 2673, 2678, 2682, 
  2687, 2691, 2695, 2700, 2704, 2709, 2713, 
  2718, 2722, 2727, 2731, 2736, 2740, 2745, 
  2749, 2753, 2758, 2762, 2767, 2771, 2776, 
  2780, 2785, 2789, 2794, 2798, 2803, 2807, 
  2812, 2816, 2821, 2825, 2830, 2834, 2839, 
  2843, 2848, 2852, 2857, 2861, 2866, 2870, 
  2875, 2879, 2884, 2888, 2893, 2897, 2902, 
  2907, 2911, 2916, 2920, 2925, 2929, 2934, 
  2938, 2943, 2947, 2952, 2957, 2961, 2966, 
  2970, 2975, 2979, 2984, 2988, 2993, 2998, 
  3002, 3007, 3011, 3016, 3020, 3025, 3030, 
  3034, 3039, 3043, 3048, 3053, 3057, 3062, 
  3066, 3071, 3076, 3080, 3085, 3089, 3094, 
  3099, 3103, 3108, 3113, 3117, 3122, 3126, 
  3131, 3136, 3140, 3145, 3150, 3154, 3159, 
  3164, 3168, 3173, 3178, 3182, 3187, 3191, 
  3196, 3201, 3206, 3210, 3215, 3220, 3224, 
  3229, 3234, 3238, 3243, 3248, 3252, 3257, 
  3262, 3267, 3271, 3276, 3281, 3285, 3290, 
  3295, 3300, 3304, 3309, 3314, 3318, 3323, 
  3328, 3333, 3337, 3342, 3347, 3352, 3356, 
  3361, 3366, 3371, 3376, 3380, 3385, 3390, 
  3395, 3399, 3404, 3409, 3414, 3419, 3423, 
  3428, 3433, 3438, 3443, 3448, 3452, 3457, 
  3462, 3467, 3472, 3477, 3481, 3486, 3491, 
  3496, 3501, 3506, 3510, 3515, 3520, 3525, 
  3530, 3535, 3540, 3545, 3549, 3554, 3559, 
  3564, 3569, 3574, 3579, 3584, 3589, 3594, 
  3599, 3603, 3608, 3613, 3618, 3623, 3628, 
  3633, 3638, 3643, 3648, 3653, 3658, 3663, 
  3668, 3673, 3678, 3683, 3688, 3693, 3698, 
  3703, 3708, 3713, 3718, 3723, 3728, 3733, 
  3738, 3743, 3748, 3753, 3758, 3763, 3768, 
  3773, 3778, 3783, 3788, 3794, 3799, 3804, 
  3809, 3814, 3819, 3824, 3829, 3834, 3839, 
  3845, 3850, 3855, 3860, 3865, 3870, 3875, 
  3880, 3886, 3891, 3896, 3901, 3906, 3911, 
  3917, 3922, 3927, 3932, 3937, 3943, 3948, 
  3953, 3958, 3963, 3969, 3974, 3979, 3984, 
  3990, 3995, 4000, 4005, 4011, 4016, 4021, 
  4026, 4032, 4037, 4042, 4048, 4053, 4058, 
  4064, 4069, 4074, 4080, 4085, 4090, 4096, 
  4101, 4106, 4112, 4117, 4122, 4128, 4133, 
  4139, 4144, 4149, 4155, 4160, 4166, 4171, 
  4176, 4182, 4187, 4193, 4198, 4204, 4209, 
  4215, 4220, 4225, 4231, 4236, 4242, 4247, 
  4253, 4258, 4264, 4270, 4275, 4281, 4286, 
  4292, 4297, 4303, 4308, 4314, 4319, 4325, 
  4331, 4336, 4342, 4347, 4353, 4359, 4364, 
  4370, 4376, 4381, 4387, 4393, 4398, 4404, 
  4410, 4415, 4421, 4427, 4432, 4438, 4444, 
  4449, 4455, 4461, 4467, 4472, 4478, 4484, 
  4490, 4496, 4501, 4507, 4513, 4519, 4524, 
  4530, 4536, 4542, 4548, 4554, 4560, 4565, 
  4571, 4577, 4583, 4589, 4595, 4601, 4607, 
  4613, 4618, 4624, 4630, 4636, 4642, 4648, 
  4654, 4660, 4666, 4672, 4678, 4684, 4690, 
  4696, 4702, 4708, 4714, 4720, 4726, 4733, 
  4739, 4745, 4751, 4757, 4763, 4769, 4775, 
  4781, 4788, 4794, 4800, 4806, 4812, 4818, 
  4825, 4831, 4837, 4843, 4850, 4856, 4862, 
  4868, 4875, 4881, 4887, 4893, 4900, 4906, 
  4912, 4919, 4925, 4931, 4938, 4944, 4951, 
  4957, 4963, 4970, 4976, 4983, 4989, 4995, 
  5002, 5008, 5015, 5021, 5028, 5034, 5041, 
  5047, 5054, 5060, 5067, 5074, 5080, 5087, 
  5093, 5100, 5106, 5113, 5120, 5126, 5133, 
  5140, 5146, 5153, 5160, 5166, 5173, 5180, 
  5187, 5193, 5200, 5207, 5214, 5221, 5227, 
  5234, 5241, 5248, 5255, 5262, 5268, 5275, 
  5282, 5289, 5296, 5303, 5310, 5317, 5324, 
  5331, 5338, 5345, 5352, 5359, 5366, 5373, 
  5380, 5387, 5394, 5401, 5408, 5416, 5423, 
  5430, 5437, 5444, 5451, 5459, 5466, 5473, 
  5480, 5487, 5495, 5502, 5509, 5517, 5524, 
  5531, 5539, 5546, 5553, 5561, 5568, 5576, 
  5583, 5590, 5598, 5605, 5613, 5620, 5628, 
  5635, 5643, 5651, 5658, 5666, 5673, 5681, 
  5689, 5696, 5704, 5712, 5719, 5727, 5735, 
  5742, 5750, 5758, 5766, 5774, 5781, 5789, 
  5797, 5805, 5813, 5821, 5829, 5837, 5845, 
  5853, 5861, 5869, 5877, 5885, 5893, 5901, 
  5909, 5917, 5925, 5933, 5941, 5949, 5958, 
  5966, 5974, 5982, 5991, 5999, 6007, 6016, 
  6024, 6032, 6041, 6049, 6057, 6066, 6074, 
  6083, 6091, 6100, 6108, 6117, 6125, 6134, 
  6143, 6151, 6160, 6169, 6177, 6186, 6195, 
  6203, 6212, 6221, 6230, 6239, 6248, 6256, 
  6265, 6274, 6283, 6292, 6301, 6310, 6319, 
  6328, 6337, 6346, 6356, 6365, 6374, 6383, 
  6392, 6401, 6411, 6420, 6429, 6439, 6448, 
  6457, 6467, 6476, 6486, 6495, 6505, 6514, 
  6524, 6533, 6543, 6553, 6562, 6572, 6582, 
  6591, 6601, 6611, 6621, 6631, 6641, 6650, 
  6660, 6670, 6680, 6690, 6700, 6711, 6721, 
  6731, 6741, 6751, 6761, 6772, 6782, 6792, 
  6802, 6813, 6823, 6834, 6844, 6855, 6865, 
  6876, 6886, 6897, 6908, 6918, 6929, 6940, 
  6951, 6961, 6972, 6983, 6994, 7005, 7016, 
  7027, 7038, 7049, 7060, 7071, 7083, 7094, 
  7105, 7117, 7128, 7139, 7151, 7162, 7174, 
  7185, 7197, 7208, 7220, 7232, 7244, 7255, 
  7267, 7279, 7291, 7303, 7315, 7327, 7339, 
  7351, 7363, 7375, 7388, 7400, 7412, 7425, 
  7437, 7449, 7462, 7475, 7487, 7500, 7513, 
  7525, 7538, 7551, 7564, 7577, 7590, 7603, 
  7616, 7629, 7642, 7655, 7669, 7682, 7695, 
  7709, 7722, 7736, 7750, 7763, 7777, 7791, 
  7805, 7819, 7833, 7847, 7861, 7875, 7889, 
  7903, 7917, 7932, 7946, 7961, 7975, 7990, 
  8005, 8019, 8034, 8049, 8064, 8079, 8094, 
  8109, 8125, 8140, 8155, 8171, 8186, 8202, 
  8217, 8233, 8249, 8265, 8281, 8297, 8313, 
  8329, 8345, 8362, 8378, 8395, 8411, 8428, 
  8445, 8461, 8478, 8495, 8512, 8530, 8547, 
  8564, 8582, 8599, 8617, 8634, 8652, 8670, 
  8688, 8706, 8725, 8743, 8761, 8780, 8798, 
  8817, 8836, 8855, 8874, 8893, 8912, 8932, 
  8951, 8971, 8991, 9010, 9030, 9050, 9071, 
  9091, 9111, 9132, 9153, 9174, 9195, 9216, 
  9237, 9258, 9280, 9301, 9323, 9345, 9367, 
  9389, 9412, 9434, 9457, 9480, 9503, 9526, 
  9549, 9573, 9596, 9620, 9644, 9668, 9693, 
  9717, 9742, 9767, 9792, 9817, 9842, 9868, 
  9894, 9920, 9946, 9973, 9999, 10026, 10053, 
  10081, 10108, 10136, 10164, 10192, 10220, 
  10249, 10278, 10307, 10337, 10366, 10396, 
  10427, 10457, 10488, 10519, 10550, 10582, 
  10614, 10646, 10679, 10712, 10745, 10779, 
  10812, 10847, 10881, 10916, 10951, 10987, 
  11023, 11060, 11096, 11134, 11171, 11209, 
  11248, 11287, 11326, 11366, 11406, 11447, 
  11488, 11529, 11572, 11614, 11658, 11702, 
  11746, 11791, 11836, 11882, 11929, 11977, 
  12025, 12073, 12123, 12173, 12224, 12275, 
  12328, 12381, 12435, 12489, 12545, 12601, 
  12659, 12717, 12776, 12837, 12898, 12960, 
  13024, 13088, 13154, 13221, 13289, 13359, 
  13430, 13502, 13576, 13651, 13728, 13806, 
  13886, 13968, 14052, 14138, 14225, 14315, 
  14407, 14501, 14598, 14697, 14799, 14903, 
  15011, 15121, 15235, 15352, 15473, 15598, 
  15727, 15860, 15997, 16140, 16288, 16441, 
  16600, 16766, 16939, 17119, 17307, 17505, 
  17712, 17929, 18158, 18400, 18656, 18928, 
  19217, 19527, 19859, 20216, 20604, 21026, 
  21488, 21999, 22569, 23211, 23943, 24793, 
  25799, 27023, 28570, 30634, 33648, 38871, 
  53811, 68751
};
 
 
 
/**
* \brief    Converts the ADC result into a temperature value.
*
*           P1 and p2 are the interpolating point just before and after the
*           ADC value. The function interpolates between these two points
*           The resulting code is very small and fast.
*           Only one integer multiplication is used.
*           The compiler can replace the division by a shift operation.
*
*           In the temperature range from -10°C to 50°C the error
*           caused by the usage of a table is 0.010°C
*
* \param    adc_value  The converted ADC result
* \return              The temperature in 0.01 °C
*
*/
float raw2temp(unsigned int adc_value){
 
  int p1, p2, p3;
  /* Estimate the interpolating point before and after the ADC value. */
  p1 = NTC_table[ (adc_value >> 1)  ];
  p2 = NTC_table[ (adc_value >> 1)+1];
 
  /* Interpolate between both points. */
  p3 =  p1 + ( (p2-p1) * (adc_value & 0x0001) ) / 2;
  
  return (float) (p3 * 0.001); // move the decimal point
};