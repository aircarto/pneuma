#define SENSOR_BASENAME "esp32-"

#define SSID_BASENAME "moduleair-"
#define HOSTNAME_BASE "moduleair-"

#define LEN_CFG_STRING 65
#define LEN_CFG_PASSWORD 65
#define LEN_CFG_STRING2 65
#define LEN_CFG_PASSWORD2 65

#define LEN_WLANSSID 35				// credentials for wifi connection
#define LEN_WWW_USERNAME 65			// credentials for basic auth of server internal website
#define LEN_FS_SSID 33				// credentials for sensor access point mode

#define LEN_HOST_CUSTOM 100
#define LEN_URL_CUSTOM 100
#define LEN_USER_CUSTOM 65
#define MAX_PORT_DIGITS 5
#define LEN_HOST_CUSTOM2 100
#define LEN_URL_CUSTOM2 100
#define LEN_USER_CUSTOM2 65
#define MAX_PORT_DIGITS2 5

// define debug levels
#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_MIN_INFO 3
#define DEBUG_MED_INFO 4
#define DEBUG_MAX_INFO 5

#define LEN_ANIMATION 30

/******************************************************************
 * Constants                                                      *
 ******************************************************************/
constexpr const unsigned long SAMPLETIME_MHZ16_MS = 10000;
constexpr const unsigned long SAMPLETIME_MHZ19_MS = 10000;

constexpr const unsigned long ONE_DAY_IN_MS = 24 * 60 * 60 * 1000;
constexpr const unsigned long DURATION_BEFORE_FORCED_RESTART_MS = ONE_DAY_IN_MS * 28;	// force a reboot every ~4 weeks

#if defined(ARDUINO_ESP32_DEV) and defined(KIT_C)
#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D12 12
#define D13 13
#define D14 14
#define D15 15
#define D16 16
#define D17 17
#define D18 18
#define D19 19
#define D21 21
#define D22 22
#define D23 23
#define D25 25
#define D26 26
#define D27 27
#define D32 32
#define D33 33
#define D34 34
#define D35 35
#define D36 36
#define D39 39
#endif
