Crazyflies
==========

圓點博士小四軸



﻿圓點博士小四軸開源軟件篇：
1,小四軸飛行器（T1套件）源碼：包括陀螺儀芯片驅動代碼，數字濾波，四元數姿態解算和電機控制代碼，此外，還包括藍牙無線傳輸代碼，NRF24L01+無線傳輸代碼，小四軸無線更新固件代碼。
2,圓點博士小四軸手持遙控器（T2套件）源碼。包括USB轉COM口代碼，藍牙編程代碼，搖桿控制代碼，和液晶屏顯示代碼。通過該代碼，用戶可以學習USB編程，藍牙編程，搖槓編程，液晶屏顯示編程能知識。
3,基於windows/Linux下的上位機代碼，能過獲取小四軸姿態，並對小四軸進行飛行控制。該代碼使用垮平台算法QT編寫。
4,基於Android的手機遙控器代碼，可以實現對小四軸的飛行控制
5,使用MPU6050 DMP來進行姿態解算的源碼。
6,對國外著名開源crazyflies開源算法的姿態部分的移植(已修改成大家熟知的MDK環境)，帶FreeRTOS操作系統。

圓點博士小四軸開源硬件篇：
1，採用了72MHz主頻的STM32F103T8控制芯片
2，採用了三軸陀螺儀和三軸加速度傳感器的MPU6050
3，集成藍牙通訊系統，可以實現小四軸和電腦/手機的通訊
4，集成鋰電池充電系統，可以使用micro-USB接口直接充電
5，預留電磁傳感器HMC5883L芯片安裝位置，方便用戶進行擴展
6，預留I2C接口，用於擴展電磁傳感器HMC5883L和高度計BMP085等模塊
7，預留SPI接口，用於擴展2.4G NRF24L01+ 無線通訊模塊
8，全機重約45克，對角電機中心點寬度120毫米左右
9，採用400mAh左右鋰電池，飛行時間6分鐘左右

圓點博士小四軸開源說明：
1，PDF格式原理圖和元器件列表,使用說明(文件名：4x_menue_v30.pdf/4x_menue_v20.pdf/4x_menue_v11.pdf/4x_menue_v10.pdf)
2，小四軸固件，含無線下載bootloader和app(文件名：src1_4x_iii_bootloader.rar/src2_4x_iii_app_release.rar）
3，遙控器固件全部源碼（文件名：src3_4x_iii_hh_release.rar）
4，PC上位機源碼和應用程序（文件名：src4_quadcopter-1.2.2.zip/app1_pc_app_ver1_2_2.zip）
5, Android的上位機全部源碼（文件名：src5_sw_android.rar/app2_sw_android.rar）
6，crazyflies開源算法的姿態部分的移植 （文件名：src6_4x_app_crazyflie_rel.rar）


https://code.jd.com/q5b170_m/etootle_4x
