#define encryptEnable 1
#define skipEncryt 0
#define useCnt 5

#define REPORT_RATE_90 0

#if REPORT_RATE_90
#define ACTIVE_TIME 445 //5S
#else
#define ACTIVE_TIME 625 //5S
#endif

#define REPORT_RATE_1000 0
#define POLL_TIME_2P4G 2

#define TOUCH_PTP_ENABLED 1
#define QDEC_CONNECT_TO_GND 0
#define ENABLE_QDEC_PULL_UP_DOWN 1
#define BUTTON_CHANGE_WORK_MODE 1               //     0:new board, 1:old board
#define AUTO_DETECT_2P4G 0
#define AUTO_PAIRING 1

#define ENABLE_NEW_AUTH 1
#define INCLUD_2_point_4G 0
#define INCLUD_BT_MODE 1
#if AUTO_DETECT_2P4G
#define ENABLE_PAIR_TO_CHANGE_WORK_MODE 0
#define ENABLE_LONG_PRESS_DIP_TO_CHANGE_WORK_MODE 0
#define CHECK_WORK_MODE_PIN_CONTINUOUSLY 0
#else
#if BUTTON_CHANGE_WORK_MODE
#define ENABLE_PAIR_TO_CHANGE_WORK_MODE 1
#define ENABLE_LONG_PRESS_DIP_TO_CHANGE_WORK_MODE 1
#define CHECK_WORK_MODE_PIN_CONTINUOUSLY 0
#else
#define ENABLE_PAIR_TO_CHANGE_WORK_MODE 0
#define ENABLE_LONG_PRESS_DIP_TO_CHANGE_WORK_MODE 0
#define CHECK_WORK_MODE_PIN_CONTINUOUSLY 1
#endif
#endif
#define UES_INTERNAL_RC_CLOCK 1
#define VOLTAGE_CHECK_ENABLE 1
#define ENABLE_RF_TEST_MODE 1
#define USING_KEY5_AS_SWITCH_MODE 0
#define SensorJustSleep 0
//#define USING_HW_SPI 0
#define NOT_USE_HW_SPI_IN_2_point_4_mode_PAW3205 0
#define NOT_USE_HW_SPI_IN_BT_mode_PAW3205 0

#define DefaultHighDPI  0

#define rightLeftPair 0

#define EnableSwitchHost 0

#define TouchEnable 0
#define DeviceID 1
#define TouchFiveButton 0

#define MyTestMode 0

#define UseLithiumBattery 1
#define CheckVddioVoltage 0

#if 0
#define Swap_XY        1   
#define Negate_X       0
#define Negate_Y       0
#endif
#define Negate_scroll  0

#define DEBUG_TRACE_ENABLE 1
#define CONSUMPTION_TEST_ENABLE 0
#define ENABLE_CLEAR_AUTH 0
#define ENABLE_AUTO_AUTH 0
#define ENABLE_AUTH_TEST 0
#define ENABLE_RTC_TIMER 0
#define ENABLE_CHANGE_CONNECTION_INTERVAL 0
#define HIGH_ACTIVE_MIN_INTERVAL   6
#define HIGH_ACTIVE_MAX_INTERVAL  6
#define LOW_ACTIVE_MIN_INTERVAL   12
#define LOW_ACTIVE_MAX_INTERVAL  12

#define CHECK_2_POINT_4_CONNECTION_TIME 500 //3s

#define SLAVE_LATENCY                      130//66//20                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */
#define TIME_TO_SLEEP_2P4                4800 // 125ms *4800=10minutes





