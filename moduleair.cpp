#include <WString.h>
#include <pgmspace.h>

#define SOFTWARE_VERSION_STR "ModuleAirArt-V1-051023"
#define SOFTWARE_VERSION_STR_SHORT "V1-051023"
String SOFTWARE_VERSION(SOFTWARE_VERSION_STR);
String SOFTWARE_VERSION_SHORT(SOFTWARE_VERSION_STR_SHORT);

#include <Arduino.h>
#include "PxMatrix.h"
#include <SPI.h>

/*****************************************************************
 * IMPORTANT                                          *
 *****************************************************************/

//On force l'utilisation des 2 SPI

//Dans SPI.cpp

// #if CONFIG_IDF_TARGET_ESP32
// SPIClass SPI(VSPI);
// SPIClass SPI_H(HSPI);
// #else
// SPIClass SPI(FSPI);
// #endif

// Dans SPI.h

// extern SPIClass SPI_H; en bas

//Dans PXMatrix

//On remplace tous les SPI. par SPI_H.

//On définit les pins:

// // HW SPI PINS
// #define SPI_BUS_CLK 14
// #define SPI_BUS_MOSI 13
// #define SPI_BUS_MISO 12
// #define SPI_BUS_SS 4

//on remplace la glcdfont.c original dans AdaFruitGFX => mod dans le dossier Fonts

/*****************************************************************
 * IMPORTANT FIN                                          *
 *****************************************************************/

#include <MHZ16_uart.h> // CO2
#include <MHZ19.h>

#include "ca-root.h"

// includes ESP32 libraries
#define FORMAT_SPIFFS_IF_FAILED true
#include <FS.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>

#if ESP_IDF_VERSION_MAJOR >= 4
#if (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(1, 0, 6))
#include "sha/sha_parallel_engine.h"
#else
#include <esp32/sha.h>
#endif
#else
//#include <hwcrypto/sha.h>
#endif

#include <WebServer.h>
#include <ESPmDNS.h>
#include <MD5Builder.h>

// includes external libraries

#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_PRINT 0
#define ARDUINOJSON_DECODE_UNICODE 0
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <StreamString.h>

// includes files
#include "./intl.h"
#include "./utils.h"
#include "defines.h"
#include "ext_def.h"
#include "html-content.h"

/*****************************************************************
 * CONFIGURATION                                          *
 *****************************************************************/
namespace cfg
{
	unsigned debug = DEBUG;

	unsigned time_for_wifi_config = TIME_FOR_WIFI_CONFIG;
	unsigned sending_intervall_ms = SENDING_INTERVALL_MS;

	char current_lang[3];

	// credentials for basic auth of internal web server
	bool www_basicauth_enabled = WWW_BASICAUTH_ENABLED;
	char www_username[LEN_WWW_USERNAME];
	char www_password[LEN_CFG_PASSWORD];

	// wifi credentials
	char wlanssid[LEN_WLANSSID];
	char wlanpwd[LEN_CFG_PASSWORD];

	// credentials of the sensor in access point mode
	char fs_ssid[LEN_FS_SSID] = FS_SSID;
	char fs_pwd[LEN_CFG_PASSWORD] = FS_PWD;

	// main config

	bool has_wifi = HAS_WIFI;

	// (in)active sensors
	bool mhz16_read = MHZ16_READ;
	bool mhz19_read = MHZ19_READ;

	// Location

	char height_above_sealevel[8] = "0";

	// send to "APIs"
	bool send2custom = SEND2CUSTOM;
	bool send2custom2 = SEND2CUSTOM2;
	bool send2csv = SEND2CSV;

	// (in)active displays
	bool has_matrix = HAS_MATRIX;

	// API AirCarto
	char host_custom[LEN_HOST_CUSTOM];
	char url_custom[LEN_URL_CUSTOM];
	bool ssl_custom = SSL_CUSTOM;
	unsigned port_custom = PORT_CUSTOM;
	char user_custom[LEN_USER_CUSTOM] = USER_CUSTOM;
	char pwd_custom[LEN_CFG_PASSWORD] = PWD_CUSTOM;

	// API AtmoSud
	char host_custom2[LEN_HOST_CUSTOM2];
	char url_custom2[LEN_URL_CUSTOM2];
	bool ssl_custom2 = SSL_CUSTOM2;
	unsigned port_custom2 = PORT_CUSTOM2;
	char user_custom2[LEN_USER_CUSTOM2] = USER_CUSTOM2;
	char pwd_custom2[LEN_CFG_PASSWORD] = PWD_CUSTOM2;

	// First load
	void initNonTrivials(const char *id)
	{
		strcpy(cfg::current_lang, CURRENT_LANG);
		strcpy_P(www_username, WWW_USERNAME);
		strcpy_P(www_password, WWW_PASSWORD);
		strcpy_P(wlanssid, WLANSSID);
		strcpy_P(wlanpwd, WLANPWD);
		strcpy_P(host_custom, HOST_CUSTOM);
		strcpy_P(url_custom, URL_CUSTOM);
		strcpy_P(host_custom2, HOST_CUSTOM2);
		strcpy_P(url_custom2, URL_CUSTOM2);

		if (!*fs_ssid)
		{
			strcpy(fs_ssid, SSID_BASENAME);
			strcat(fs_ssid, id);
		}
	}
}

bool spiffs_matrix;

// define size of the config JSON
#define JSON_BUFFER_SIZE 2300
// define size of the AtmoSud Forecast API JSON
#define JSON_BUFFER_SIZE2 500

LoggerConfig loggerConfigs[LoggerCount];

// test variables
long int sample_count = 0;
bool moduleair_selftest_failed = false;

WebServer server(80);

// include JSON config reader
#include "./moduleair-cfg.h"

/*****************************************************************
 * Display definitions                                           *
 *****************************************************************/

//For the matrix
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define matrix_width 64
//#define matrix_height 32
#define matrix_height 64
uint8_t display_draw_time = 30; //10-50 is usually fine
PxMATRIX display(matrix_width, matrix_height, P_LAT, P_OE, P_A, P_B, P_C, P_D, P_E);

void IRAM_ATTR display_updater()
{
	// Increment the counter and set the time of ISR
	portENTER_CRITICAL_ISR(&timerMux);
	display.display(display_draw_time);
	portEXIT_CRITICAL_ISR(&timerMux);
}

void display_update_enable(bool is_enable)
{
	Debug.print("Call display_update_enable function with:");
	if (is_enable)
	{
		Debug.println("true");
		//timer = timerBegin(0, 80, true);
		timerAttachInterrupt(timer, &display_updater, true);
		timerAlarmWrite(timer, 4000, true);
		timerAlarmEnable(timer);
	}
	else
	{
		Debug.println("false");
		timerDetachInterrupt(timer);
		timerAlarmDisable(timer);
	}
}

void drawImage(int x, int y, int h, int w, const uint16_t image[])
{
	int imageHeight = h;
	int imageWidth = w;
	int counter = 0;
	for (int yy = 0; yy < imageHeight; yy++)
	{
		for (int xx = 0; xx < imageWidth; xx++)
		{
			display.drawPixel(xx + x, yy + y, image[counter]);
			counter++;
		}
	}
}

void drawRandomPixel(int x, int y, int h, int w)
{
	int imageHeight = h;
	int imageWidth = w;
	int counter = 0;
	for (int yy = 0; yy < imageHeight; yy++)
	{
		for (int xx = 0; xx < imageWidth; xx++)
		{
			display.drawPixel(xx + x, yy + y, rand() % 65536);
		}
	}
}

/*****************************************************************
 * Serial declarations                                           *
 *****************************************************************/

#define serialMHZ (Serial2)

/*****************************************************************
 * Activation Screen                                          *
 *****************************************************************/

bool cfg_screen_co2 = true;

/*****************************************************************
 * MH-Z16 declaration                                        *
 *****************************************************************/
MHZ16_uart mhz16;

/*****************************************************************
 * MH-Z19 declaration                                        *
 *****************************************************************/
MHZ19 mhz19;

/*****************************************************************
 * Time                                       *
 *****************************************************************/

// time management varialbles
bool send_now = false;
unsigned long starttime;
unsigned long time_end_setup;
unsigned long time_before_config;
unsigned long time_animation;
unsigned long time_point_device_start_ms;
unsigned long starttime_MHZ16;
unsigned long starttime_MHZ19;
unsigned long act_micro;
unsigned long act_milli;
unsigned long last_micro = 0;
unsigned long min_micro = 1000000000;
unsigned long max_micro = 0;

unsigned long sending_time = 0;
unsigned long last_update_attempt;
int last_sendData_returncode;

bool wifi_connection_lost;

/*****************************************************************
 * Data variables                                      *
 *****************************************************************/
float last_value_MHZ16 = -1.0;
uint32_t mhz16_sum = 0;
uint16_t mhz16_val_count = 0;

float last_value_MHZ19 = -1.0;
uint32_t mhz19_sum = 0;
uint16_t mhz19_val_count = 0;

String last_data_string;
int last_signal_strength;
int last_disconnect_reason;

String esp_chipid;

unsigned long MHZ16_error_count;
unsigned long MHZ19_error_count;
unsigned long WiFi_error_count;

unsigned long last_page_load = millis();

bool wificonfig_loop = false;
uint8_t sntp_time_set;

unsigned long count_sends = 0;
unsigned long last_display_millis_oled = 0;
unsigned long last_display_millis_matrix = 0;
uint8_t next_display_count = 0;

struct struct_wifiInfo
{
	char ssid[LEN_WLANSSID];
	uint8_t encryptionType;
	int32_t RSSI;
	int32_t channel;
};

struct struct_wifiInfo *wifiInfo;
uint8_t count_wifiInfo;

#define msSince(timestamp_before) (act_milli - (timestamp_before))

const char data_first_part[] PROGMEM = "{\"software_version\": \"" SOFTWARE_VERSION_STR "\", \"sensordatavalues\":[";
const char JSON_SENSOR_DATA_VALUES[] PROGMEM = "sensordatavalues";

/*****************************************************************
 * write config to spiffs                                        *
 *****************************************************************/
static bool writeConfig()
{

	Debug.print("cfg::has_matrix: ");
	Debug.println(cfg::has_matrix);

	if (cfg::has_matrix && spiffs_matrix)
	{
		display_update_enable(false); //prevent crash
	}

	DynamicJsonDocument json(JSON_BUFFER_SIZE);
	debug_outln_info(F("Saving config..."));
	json["SOFTWARE_VERSION"] = SOFTWARE_VERSION;

	for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
	{
		ConfigShapeEntry c;
		memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
		switch (c.cfg_type)
		{
		case Config_Type_Bool:
			json[c.cfg_key()].set(*c.cfg_val.as_bool);
			break;
		case Config_Type_UInt:
		case Config_Type_Time:
			json[c.cfg_key()].set(*c.cfg_val.as_uint);
			break;
		case Config_Type_Password:
		case Config_Type_Hex:
		case Config_Type_String:
			json[c.cfg_key()].set(c.cfg_val.as_str);
			break;
		};
	}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

	SPIFFS.remove(F("/config.json.old"));
	SPIFFS.rename(F("/config.json"), F("/config.json.old"));

	File configFile = SPIFFS.open(F("/config.json"), "w");
	if (configFile)
	{
		serializeJsonPretty(json, Debug);
		serializeJson(json, configFile);
		configFile.close();
		debug_outln_info(F("Config written successfully."));
	}
	else
	{
		debug_outln_error(F("failed to open config file for writing"));
		return false;
	}

#pragma GCC diagnostic pop

	return true;
}

/*****************************************************************
 * read config from spiffs                                       *
 *****************************************************************/

/* backward compatibility for the times when we stored booleans as strings */
static bool boolFromJSON(const DynamicJsonDocument &json, const __FlashStringHelper *key)
{
	if (json[key].is<const char *>())
	{
		return !strcmp_P(json[key].as<const char *>(), PSTR("true"));
	}
	return json[key].as<bool>();
}

static void readConfig(bool oldconfig = false)
{
	bool rewriteConfig = false;

	String cfgName(F("/config.json"));
	if (oldconfig)
	{
		cfgName += F(".old");
	}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
	File configFile = SPIFFS.open(cfgName, "r");
	if (!configFile)
	{
		if (!oldconfig)
		{
			return readConfig(true /* oldconfig */);
		}

		debug_outln_error(F("failed to open config file."));
		return;
	}

	debug_outln_info(F("opened config file..."));
	DynamicJsonDocument json(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json, configFile.readString());
	configFile.close();
#pragma GCC diagnostic pop

	if (!err)
	{
		serializeJsonPretty(json, Debug);
		debug_outln_info(F("parsed json..."));
		for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
		{
			ConfigShapeEntry c;
			memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
			if (json[c.cfg_key()].isNull())
			{
				continue;
			}
			switch (c.cfg_type)
			{
			case Config_Type_Bool:
				*(c.cfg_val.as_bool) = boolFromJSON(json, c.cfg_key());
				break;
			case Config_Type_UInt:
			case Config_Type_Time:
				*(c.cfg_val.as_uint) = json[c.cfg_key()].as<unsigned int>();
				break;
			case Config_Type_String:
			case Config_Type_Hex:
			case Config_Type_Password:
				strncpy(c.cfg_val.as_str, json[c.cfg_key()].as<const char *>(), c.cfg_len);
				c.cfg_val.as_str[c.cfg_len] = '\0';
				break;
			};
		}
		String writtenVersion(json["SOFTWARE_VERSION"].as<const char *>());
		if (writtenVersion.length() && writtenVersion[0] == 'N' && SOFTWARE_VERSION != writtenVersion)
		{
			debug_outln_info(F("Rewriting old config from: "), writtenVersion);
			// would like to do that, but this would wipe firmware.old which the two stage loader
			// might still need
			// SPIFFS.format();
			rewriteConfig = true;
		}
	}
	else
	{
		debug_outln_error(F("failed to load json config"));

		if (!oldconfig)
		{
			return readConfig(true /* oldconfig */);
		}
	}

	if (rewriteConfig)
	{
		writeConfig();
	}
}

static void init_config()
{

	debug_outln_info(F("mounting FS..."));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

	bool spiffs_begin_ok = SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED);

#pragma GCC diagnostic pop

	if (!spiffs_begin_ok)
	{
		debug_outln_error(F("failed to mount FS"));
		return;
	}
	readConfig();
}

/*****************************************************************
 * Prepare information for data Loggers                          *
 *****************************************************************/
static void createLoggerConfigs()
{
	loggerConfigs[LoggerCustom].destport = cfg::port_custom;
	if (cfg::send2custom && cfg::ssl_custom)
	{
		loggerConfigs[LoggerCustom].destport = 443;
	}
	loggerConfigs[LoggerCustom2].destport = cfg::port_custom2;
	if (cfg::send2custom2 && cfg::ssl_custom2)
	{
		loggerConfigs[LoggerCustom2].destport = 443;
	}
}

/*****************************************************************
 * html helper functions                                         *
 *****************************************************************/
static void start_html_page(String &page_content, const String &title)
{
	last_page_load = millis();

	RESERVE_STRING(s, LARGE_STR);
	s = FPSTR(WEB_PAGE_HEADER);
	s.replace("{t}", title);
	server.setContentLength(CONTENT_LENGTH_UNKNOWN);
	server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), s);

	server.sendContent_P(WEB_PAGE_HEADER_HEAD);

	s = FPSTR(WEB_PAGE_HEADER_BODY);
	s.replace("{t}", title);
	if (title != " ")
	{
		s.replace("{n}", F("&raquo;"));
	}
	else
	{
		s.replace("{n}", emptyString);
	}
	s.replace("{id}", esp_chipid);
	// s.replace("{macid}", esp_mac_id);
	// s.replace("{mac}", WiFi.macAddress());
	page_content += s;
}

static void end_html_page(String &page_content)
{
	if (page_content.length())
	{
		server.sendContent(page_content);
	}
	server.sendContent_P(WEB_PAGE_FOOTER);
}

static void add_form_input(String &page_content, const ConfigShapeId cfgid, const __FlashStringHelper *info, const int length)
{
	RESERVE_STRING(s, MED_STR);
	s = F("<tr>"
		  "<td title='[&lt;= {l}]'>{i}:&nbsp;</td>"
		  "<td style='width:{l}em'>"
		  "<input type='{t}' name='{n}' id='{n}' placeholder='{i}' value='{v}' maxlength='{l}'/>"
		  "</td></tr>");
	String t_value;
	ConfigShapeEntry c;
	memcpy_P(&c, &configShape[cfgid], sizeof(ConfigShapeEntry));
	switch (c.cfg_type)
	{
	case Config_Type_UInt:
		t_value = String(*c.cfg_val.as_uint);
		s.replace("{t}", F("number"));
		break;
	case Config_Type_Time:
		t_value = String((*c.cfg_val.as_uint) / 1000);
		s.replace("{t}", F("number"));
		break;
	case Config_Type_Password:
		s.replace("{t}", F("text"));
		info = FPSTR(INTL_PASSWORD);
	case Config_Type_Hex:
		s.replace("{t}", F("hex"));
	default:
		t_value = c.cfg_val.as_str;
		t_value.replace("'", "&#39;");
		s.replace("{t}", F("text"));
	}
	s.replace("{i}", info);
	s.replace("{n}", String(c.cfg_key()));
	s.replace("{v}", t_value);
	s.replace("{l}", String(length));
	page_content += s;
}

static String form_checkbox(const ConfigShapeId cfgid, const String &info, const bool linebreak)
{
	RESERVE_STRING(s, MED_STR);
	s = F("<label for='{n}'>"
		  "<input type='checkbox' name='{n}' value='1' id='{n}' {c}/>"
		  "<input type='hidden' name='{n}' value='0'/>"
		  "{i}</label><br/>");
	if (*configShape[cfgid].cfg_val.as_bool)
	{
		s.replace("{c}", F(" checked='checked'"));
	}
	else
	{
		s.replace("{c}", emptyString);
	};
	s.replace("{i}", info);
	s.replace("{n}", String(configShape[cfgid].cfg_key()));
	if (!linebreak)
	{
		s.replace("<br/>", emptyString);
	}
	return s;
}

static String form_submit(const String &value)
{
	String s = F("<tr>"
				 "<td>&nbsp;</td>"
				 "<td>"
				 "<input type='submit' name='submit' value='{v}' />"
				 "</td>"
				 "</tr>");
	s.replace("{v}", value);
	return s;
}

static String form_select_lang()
{
	String s_select = F(" selected='selected'");
	String s = F("<tr>"
				 "<td>" INTL_LANGUAGE ":&nbsp;</td>"
				 "<td>"
				 "<select id='current_lang' name='current_lang'>"
				 "<option value='FR'>Français (FR)</option>"
				 "<option value='EN'>English (EN)</option>"
				 "</select>"
				 "</td>"
				 "</tr>");

	s.replace("'" + String(cfg::current_lang) + "'>", "'" + String(cfg::current_lang) + "'" + s_select + ">");
	return s;
}

static void add_warning_first_cycle(String &page_content)
{
	String s = FPSTR(INTL_TIME_TO_FIRST_MEASUREMENT);
	unsigned int time_to_first = cfg::sending_intervall_ms - msSince(starttime);
	if (time_to_first > cfg::sending_intervall_ms)
	{
		time_to_first = 0;
	}
	s.replace("{v}", String(((time_to_first + 500) / 1000)));
	page_content += s;
}

static void add_age_last_values(String &s)
{
	s += "<b>";
	unsigned int time_since_last = msSince(starttime);
	if (time_since_last > cfg::sending_intervall_ms)
	{
		time_since_last = 0;
	}
	s += String((time_since_last + 500) / 1000);
	s += FPSTR(INTL_TIME_SINCE_LAST_MEASUREMENT);
	s += FPSTR(WEB_B_BR_BR);
}

/*****************************************************************
 * Webserver request auth: prompt for BasicAuth
 *
 * -Provide BasicAuth for all page contexts except /values and images
 *****************************************************************/
static bool webserver_request_auth()
{
	if (cfg::www_basicauth_enabled && !wificonfig_loop)
	{
		debug_outln_info(F("validate request auth..."));
		if (!server.authenticate(cfg::www_username, cfg::www_password))
		{
			server.requestAuthentication(BASIC_AUTH, "Sensor Login", F("Authentication failed"));
			return false;
		}
	}
	return true;
}

static void sendHttpRedirect()
{
	server.sendHeader(F("Location"), F("http://192.168.4.1/config"));
	server.send(302, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), emptyString);
}

/*****************************************************************
 * Webserver root: show all options                              *
 *****************************************************************/
static void webserver_root()
{
	// Reactivate the interrupt to turn on the matrix if the user return to homepage

	if (cfg::has_matrix)
	{
		display_update_enable(true);
	}

	if (WiFi.status() != WL_CONNECTED)
	{
		sendHttpRedirect();
	}
	else
	{
		if (!webserver_request_auth())
		{
			return;
		}

		RESERVE_STRING(page_content, XLARGE_STR);
		start_html_page(page_content, emptyString);
		debug_outln_info(F("ws: root ..."));

		// Enable Pagination
		page_content += FPSTR(WEB_ROOT_PAGE_CONTENT);
		page_content.replace(F("{t}"), FPSTR(INTL_CURRENT_DATA));
		page_content.replace(F("{s}"), FPSTR(INTL_DEVICE_STATUS));
		page_content.replace(F("{conf}"), FPSTR(INTL_CONFIGURATION));
		page_content.replace(F("{restart}"), FPSTR(INTL_RESTART_SENSOR));
		page_content.replace(F("{debug}"), FPSTR(INTL_DEBUG_LEVEL));
		end_html_page(page_content);
	}
}

/*****************************************************************
 * Webserver config: show config page                            *
 *****************************************************************/
static void webserver_config_send_body_get(String &page_content)
{
	auto add_form_checkbox = [&page_content](const ConfigShapeId cfgid, const String &info)
	{
		page_content += form_checkbox(cfgid, info, true);
	};

	auto add_form_checkbox_sensor = [&add_form_checkbox](const ConfigShapeId cfgid, __const __FlashStringHelper *info)
	{
		add_form_checkbox(cfgid, add_sensor_type(info));
	};

	debug_outln_info(F("begin webserver_config_body_get ..."));
	page_content += F("<form method='POST' action='/config' style='width:100%;'>\n"
					  "<input class='radio' id='r1' name='group' type='radio' checked>"
					  "<input class='radio' id='r2' name='group' type='radio'>"
					  "<input class='radio' id='r3' name='group' type='radio'>"
					  "<input class='radio' id='r4' name='group' type='radio'>"
					  "<div class='tabs'>"
					  "<label class='tab' id='tab1' for='r1'>" INTL_WIFI_SETTINGS "</label>"
					  "<label class='tab' id='tab2' for='r2'>");
	page_content += FPSTR(INTL_MORE_SETTINGS);
	page_content += F("</label>"
					  "<label class='tab' id='tab3' for='r3'>");
	page_content += FPSTR(INTL_SENSORS);
	page_content += F(
		"</label>"
		"<label class='tab' id='tab4' for='r4'>APIs");
	page_content += F("</label></div><div class='panels'>"
					  "<div class='panel' id='panel1'>");

	if (wificonfig_loop)
	{ // scan for wlan ssids
		page_content += F("<div id='wifilist'>" INTL_WIFI_NETWORKS "</div><br/>");
	}
	add_form_checkbox(Config_has_wifi, FPSTR(INTL_WIFI_ACTIVATION));
	page_content += FPSTR(TABLE_TAG_OPEN);
	add_form_input(page_content, Config_wlanssid, FPSTR(INTL_FS_WIFI_NAME), LEN_WLANSSID - 1);
	add_form_input(page_content, Config_wlanpwd, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	page_content += F("<hr/>\n<br/><b>");

	page_content += FPSTR(INTL_AB_HIER_NUR_ANDERN);
	page_content += FPSTR(WEB_B_BR);
	page_content += FPSTR(BR_TAG);

	// Paginate page after ~ 1500 Bytes
	server.sendContent(page_content);
	page_content = emptyString;

	add_form_checkbox(Config_www_basicauth_enabled, FPSTR(INTL_BASICAUTH));
	page_content += FPSTR(TABLE_TAG_OPEN);
	add_form_input(page_content, Config_www_username, FPSTR(INTL_USER), LEN_WWW_USERNAME - 1);
	add_form_input(page_content, Config_www_password, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	page_content += FPSTR(BR_TAG);

	// Paginate page after ~ 1500 Bytes
	server.sendContent(page_content);

	if (!wificonfig_loop)
	{
		page_content = FPSTR(INTL_FS_WIFI_DESCRIPTION);
		page_content += FPSTR(BR_TAG);

		page_content += FPSTR(TABLE_TAG_OPEN);
		add_form_input(page_content, Config_fs_ssid, FPSTR(INTL_FS_WIFI_NAME), LEN_FS_SSID - 1);
		add_form_input(page_content, Config_fs_pwd, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
		page_content += FPSTR(TABLE_TAG_CLOSE_BR);

		// Paginate page after ~ 1500 Bytes
		server.sendContent(page_content);
	}
	page_content = tmpl(FPSTR(WEB_DIV_PANEL), String(2));
	page_content += F("<b>" INTL_DISPLAY "</b>&nbsp;");
	page_content += FPSTR(WEB_B_BR);
	add_form_checkbox(Config_has_matrix, FPSTR(INTL_MATRIX));
	server.sendContent(page_content);
	page_content = FPSTR(WEB_BR_LF_B);
	page_content += F(INTL_FIRMWARE "</b>&nbsp;");

	page_content += FPSTR(TABLE_TAG_OPEN);
	add_form_input(page_content, Config_debug, FPSTR(INTL_DEBUG_LEVEL), 1);
	add_form_input(page_content, Config_sending_intervall_ms, FPSTR(INTL_MEASUREMENT_INTERVAL), 5);
	add_form_input(page_content, Config_time_for_wifi_config, FPSTR(INTL_DURATION_ROUTER_MODE), 5);
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	server.sendContent(page_content);

	page_content = tmpl(FPSTR(WEB_DIV_PANEL), String(3));

	page_content += FPSTR("<b>");
	page_content += FPSTR(INTL_CO2_SENSORS);
	page_content += FPSTR(WEB_B_BR);

	add_form_checkbox_sensor(Config_mhz16_read, FPSTR(INTL_MHZ16));
	add_form_checkbox_sensor(Config_mhz19_read, FPSTR(INTL_MHZ19));

	// // Paginate page after ~ 1500 Bytes
	server.sendContent(page_content);
	page_content = emptyString;

	page_content = tmpl(FPSTR(WEB_DIV_PANEL), String(4));

	page_content += FPSTR("<b>");
	page_content += FPSTR(INTL_SEND_TO);
	page_content += FPSTR(WEB_B_BR);

	add_form_checkbox(Config_send2csv, FPSTR(WEB_CSV));

	server.sendContent(page_content);
	page_content = emptyString;

	page_content += FPSTR(BR_TAG);
	page_content += form_checkbox(Config_send2custom, FPSTR(INTL_SEND_TO_OWN_API), false);
	page_content += FPSTR(WEB_NBSP_NBSP_BRACE);
	page_content += form_checkbox(Config_ssl_custom, FPSTR(WEB_HTTPS), false);
	page_content += FPSTR(WEB_BRACE_BR);

	server.sendContent(page_content);
	page_content = FPSTR(TABLE_TAG_OPEN);
	add_form_input(page_content, Config_host_custom, FPSTR(INTL_SERVER), LEN_HOST_CUSTOM - 1);
	add_form_input(page_content, Config_url_custom, FPSTR(INTL_PATH), LEN_URL_CUSTOM - 1);
	add_form_input(page_content, Config_port_custom, FPSTR(INTL_PORT), MAX_PORT_DIGITS);
	add_form_input(page_content, Config_user_custom, FPSTR(INTL_USER), LEN_USER_CUSTOM - 1);
	add_form_input(page_content, Config_pwd_custom, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);

	page_content += FPSTR(BR_TAG);
	page_content += form_checkbox(Config_send2custom2, FPSTR(INTL_SEND_TO_OWN_API2), false);
	page_content += FPSTR(WEB_NBSP_NBSP_BRACE);
	page_content += form_checkbox(Config_ssl_custom2, FPSTR(WEB_HTTPS), false);
	page_content += FPSTR(WEB_BRACE_BR);

	server.sendContent(page_content);
	page_content = emptyString;
	page_content = FPSTR(TABLE_TAG_OPEN);
	add_form_input(page_content, Config_host_custom2, FPSTR(INTL_SERVER2), LEN_HOST_CUSTOM2 - 1);
	add_form_input(page_content, Config_url_custom2, FPSTR(INTL_PATH2), LEN_URL_CUSTOM2 - 1);
	add_form_input(page_content, Config_port_custom2, FPSTR(INTL_PORT2), MAX_PORT_DIGITS2);
	add_form_input(page_content, Config_user_custom2, FPSTR(INTL_USER2), LEN_USER_CUSTOM2 - 1);
	add_form_input(page_content, Config_pwd_custom2, FPSTR(INTL_PASSWORD2), LEN_CFG_PASSWORD2 - 1);
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);

	page_content += F("</div></div>");
	page_content += form_submit(FPSTR(INTL_SAVE_AND_RESTART));
	page_content += FPSTR(BR_TAG);
	page_content += FPSTR(WEB_BR_FORM);
	if (wificonfig_loop)
	{ // scan for wlan ssids
		page_content += F("<script>window.setTimeout(load_wifi_list,1000);</script>");
	}
	server.sendContent(page_content);
	page_content = emptyString;
}

static void webserver_config_send_body_post(String &page_content)
{
	String masked_pwd;

	for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
	{
		ConfigShapeEntry c;
		memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
		const String s_param(c.cfg_key());
		if (!server.hasArg(s_param))
		{
			continue;
		}
		const String server_arg(server.arg(s_param));

		switch (c.cfg_type)
		{
		case Config_Type_UInt:
			*(c.cfg_val.as_uint) = server_arg.toInt();
			break;
		case Config_Type_Time:
			*(c.cfg_val.as_uint) = server_arg.toInt() * 1000;
			break;
		case Config_Type_Bool:
			*(c.cfg_val.as_bool) = (server_arg == "1");
			break;
		case Config_Type_String:
			strncpy(c.cfg_val.as_str, server_arg.c_str(), c.cfg_len);
			c.cfg_val.as_str[c.cfg_len] = '\0';
			break;
		case Config_Type_Password:
			if (server_arg.length())
			{
				server_arg.toCharArray(c.cfg_val.as_str, LEN_CFG_PASSWORD);
			}
			break;
		case Config_Type_Hex:
			strncpy(c.cfg_val.as_str, server_arg.c_str(), c.cfg_len);
			c.cfg_val.as_str[c.cfg_len] = '\0';
			break;
		}
	}

	page_content += FPSTR(INTL_SENSOR_IS_REBOOTING);

	server.sendContent(page_content);
	page_content = emptyString;
}

static void sensor_restart()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

	SPIFFS.end();

#pragma GCC diagnostic pop

	debug_outln_info(F("Restart."));
	delay(500);
	ESP.restart();
	// should not be reached
	while (true)
	{
		yield();
	}
}

static void webserver_config()
{
	if (WiFi.getMode() == WIFI_MODE_STA)
	{
		debug_outln_info(F("STA"));
		if (cfg::has_matrix)
		{
			display_update_enable(false);
		}
	}

	if (WiFi.getMode() == WIFI_MODE_AP)
	{
		debug_outln_info(F("AP"));
	}

	if (!webserver_request_auth())
	{
		return;
	}

	debug_outln_info(F("ws: config page ..."));

	server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
	server.sendHeader(F("Pragma"), F("no-cache"));
	server.sendHeader(F("Expires"), F("0"));
	// Enable Pagination (Chunked Transfer)
	server.setContentLength(CONTENT_LENGTH_UNKNOWN);

	RESERVE_STRING(page_content, XLARGE_STR);

	start_html_page(page_content, FPSTR(INTL_CONFIGURATION));
	if (wificonfig_loop)
	{ // scan for wlan ssids
		page_content += FPSTR(WEB_CONFIG_SCRIPT);
	}

	if (server.method() == HTTP_GET)
	{
		webserver_config_send_body_get(page_content);
	}
	else
	{
		webserver_config_send_body_post(page_content);
	}
	end_html_page(page_content);

	if (server.method() == HTTP_POST)
	{
		if (writeConfig())
		{
			sensor_restart();
		}
	}
}

/*****************************************************************
 * Webserver wifi: show available wifi networks                  *
 *****************************************************************/
static void webserver_wifi()
{
	String page_content;

	debug_outln_info(F("wifi networks found: "), String(count_wifiInfo));
	if (count_wifiInfo == 0)
	{
		page_content += FPSTR(BR_TAG);
		page_content += FPSTR(INTL_NO_NETWORKS);
		page_content += FPSTR(BR_TAG);
	}
	else
	{
		std::unique_ptr<int[]> indices(new int[count_wifiInfo]);
		debug_outln_info(F("ws: wifi ..."));
		for (unsigned i = 0; i < count_wifiInfo; ++i)
		{
			indices[i] = i;
		}
		for (unsigned i = 0; i < count_wifiInfo; i++)
		{
			for (unsigned j = i + 1; j < count_wifiInfo; j++)
			{
				if (wifiInfo[indices[j]].RSSI > wifiInfo[indices[i]].RSSI)
				{
					std::swap(indices[i], indices[j]);
				}
			}
		}
		int duplicateSsids = 0;
		for (int i = 0; i < count_wifiInfo; i++)
		{
			if (indices[i] == -1)
			{
				continue;
			}
			for (int j = i + 1; j < count_wifiInfo; j++)
			{
				if (strncmp(wifiInfo[indices[i]].ssid, wifiInfo[indices[j]].ssid, sizeof(wifiInfo[0].ssid)) == 0)
				{
					indices[j] = -1; // set dup aps to index -1
					++duplicateSsids;
				}
			}
		}

		page_content += FPSTR(INTL_NETWORKS_FOUND);
		page_content += String(count_wifiInfo - duplicateSsids);
		page_content += FPSTR(BR_TAG);
		page_content += FPSTR(BR_TAG);
		page_content += FPSTR(TABLE_TAG_OPEN);
		// if (n > 30) n=30;
		for (int i = 0; i < count_wifiInfo; ++i)
		{
			if (indices[i] == -1)
			{
				continue;
			}
			// Print SSID and RSSI for each network found
			page_content += wlan_ssid_to_table_row(wifiInfo[indices[i]].ssid, ((wifiInfo[indices[i]].encryptionType == WIFI_AUTH_OPEN) ? " " : u8"🔒"), wifiInfo[indices[i]].RSSI);
		}
		page_content += FPSTR(TABLE_TAG_CLOSE_BR);
		page_content += FPSTR(BR_TAG);
	}
	server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), page_content);
}

/*****************************************************************
 * Webserver root: show latest values                            *
 *****************************************************************/
static void webserver_values()
{

	if (WiFi.status() != WL_CONNECTED)
	{
		sendHttpRedirect();
		return;
	}

	RESERVE_STRING(page_content, XLARGE_STR);
	start_html_page(page_content, FPSTR(INTL_CURRENT_DATA));
	const String unit_Deg("°");
	const String unit_P("hPa");
	const String unit_T("°C");
	const String unit_CO2("ppm");
	const String unit_COV("ppb");
	const String unit_NC();
	const String unit_LA(F("dB(A)"));
	float dew_point_temp;

	const int signal_quality = calcWiFiSignalQuality(last_signal_strength);
	debug_outln_info(F("ws: values ..."));
	if (!count_sends)
	{
		page_content += F("<b style='color:red'>");
		add_warning_first_cycle(page_content);
		page_content += FPSTR(WEB_B_BR_BR);
	}
	else
	{
		add_age_last_values(page_content);
	}

	auto add_table_pm_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float &value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0), F("µg/m³"));
	};

	auto add_table_nc_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0), F("#/cm³"));
	};

	auto add_table_t_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -128, 1, 0), "°C");
	};

	auto add_table_h_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0), "%");
	};

	auto add_table_co2_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float &value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0).substring(0, check_display_value(value, -1, 1, 0).indexOf(".")), "ppm"); //remove after .
	};

	auto add_table_voc_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float &value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0).substring(0, check_display_value(value, -1, 1, 0).indexOf(".")), "ppb"); //remove after .
	};

	auto add_table_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const String &value, const String &unit)
	{
		add_table_row_from_value(page_content, sensor, param, value, unit);
	};

	server.sendContent(page_content);
	page_content = F("<table cellspacing='0' cellpadding='5' class='v'>\n"
					 "<thead><tr><th>" INTL_SENSOR "</th><th> " INTL_PARAMETER "</th><th>" INTL_VALUE "</th></tr></thead>");

	if (cfg::mhz16_read)
	{
		const char *const sensor_name = SENSORS_MHZ16;
		add_table_co2_value(FPSTR(sensor_name), FPSTR(INTL_CO2), last_value_MHZ16);
		page_content += FPSTR(EMPTY_ROW);
	}

	if (cfg::mhz19_read)
	{
		const char *const sensor_name = SENSORS_MHZ19;
		add_table_co2_value(FPSTR(sensor_name), FPSTR(INTL_CO2), last_value_MHZ19);
		page_content += FPSTR(EMPTY_ROW);
	}

	server.sendContent(page_content);
	page_content = emptyString;

	add_table_value(F("WiFi"), FPSTR(INTL_SIGNAL_STRENGTH), String(last_signal_strength), "dBm");
	add_table_value(F("WiFi"), FPSTR(INTL_SIGNAL_QUALITY), String(signal_quality), "%");

	page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	page_content += FPSTR(BR_TAG);
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver root: show device status
 *****************************************************************/
static void webserver_status()
{
	if (WiFi.status() != WL_CONNECTED)
	{
		sendHttpRedirect();
		return;
	}

	RESERVE_STRING(page_content, XLARGE_STR);
	start_html_page(page_content, FPSTR(INTL_DEVICE_STATUS));

	debug_outln_info(F("ws: status ..."));
	server.sendContent(page_content);
	page_content = F("<table cellspacing='0' cellpadding='5' class='v'>\n"
					 "<thead><tr><th> " INTL_PARAMETER "</th><th>" INTL_VALUE "</th></tr></thead>");
	String versionHtml(SOFTWARE_VERSION);
	versionHtml += F("/ST:");
	versionHtml += String(!moduleair_selftest_failed);
	versionHtml += '/';
	versionHtml.replace("/", FPSTR(BR_TAG));
	add_table_row_from_value(page_content, FPSTR(INTL_FIRMWARE), versionHtml);
	add_table_row_from_value(page_content, F("Free Memory"), String(ESP.getFreeHeap()));
	time_t now = time(nullptr);
	add_table_row_from_value(page_content, FPSTR(INTL_TIME_UTC), ctime(&now));
	add_table_row_from_value(page_content, F("Uptime"), delayToString(millis() - time_point_device_start_ms));
	page_content += FPSTR(EMPTY_ROW);
	page_content += F("<tr><td colspan='2'><b>" INTL_ERROR "</b></td></tr>");
	String wifiStatus(WiFi_error_count);
	wifiStatus += '/';
	wifiStatus += String(last_signal_strength);
	wifiStatus += '/';
	wifiStatus += String(last_disconnect_reason);
	add_table_row_from_value(page_content, F("WiFi"), wifiStatus);

	// if (last_update_returncode != 0)
	// {
	// 	add_table_row_from_value(page_content, F("OTA Return"),
	// 							 last_update_returncode > 0 ? String(last_update_returncode) : HTTPClient::errorToString(last_update_returncode));
	// }
	for (unsigned int i = 0; i < LoggerCount; ++i)
	{
		if (loggerConfigs[i].errors)
		{
			const __FlashStringHelper *logger = loggerDescription(i);
			if (logger)
			{
				add_table_row_from_value(page_content, logger, String(loggerConfigs[i].errors));
			}
		}
	}

	if (last_sendData_returncode != 0)
	{
		add_table_row_from_value(page_content, F("Data Send Return"),
								 last_sendData_returncode > 0 ? String(last_sendData_returncode) : HTTPClient::errorToString(last_sendData_returncode));
	}
	if (cfg::mhz16_read)
	{
		add_table_row_from_value(page_content, FPSTR(SENSORS_MHZ16), String(MHZ16_error_count));
	}
	if (cfg::mhz19_read)
	{
		add_table_row_from_value(page_content, FPSTR(SENSORS_MHZ19), String(MHZ19_error_count));
	}
	server.sendContent(page_content);
	page_content = emptyString;

	if (count_sends > 0)
	{
		page_content += FPSTR(EMPTY_ROW);
		add_table_row_from_value(page_content, F(INTL_NUMBER_OF_MEASUREMENTS), String(count_sends));
		if (sending_time > 0)
		{
			add_table_row_from_value(page_content, F(INTL_TIME_SENDING_MS), String(sending_time), "ms");
		}
	}

	page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver read serial ring buffer                             *
 *****************************************************************/
static void webserver_serial()
{
	String s(Debug.popLines());

	server.send(s.length() ? 200 : 204, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), s);
}

/*****************************************************************
 * Webserver set debug level                                     *
 *****************************************************************/
static void webserver_debug_level()
{
	if (!webserver_request_auth())
	{
		return;
	}

	RESERVE_STRING(page_content, LARGE_STR);
	start_html_page(page_content, FPSTR(INTL_DEBUG_LEVEL));

	if (server.hasArg("lvl"))
	{
		debug_outln_info(F("ws: debug level ..."));

		const int lvl = server.arg("lvl").toInt();
		if (lvl >= 0 && lvl <= 5)
		{
			cfg::debug = lvl;
			page_content += F("<h3>");
			page_content += FPSTR(INTL_DEBUG_SETTING_TO);
			page_content += ' ';

			const __FlashStringHelper *lvlText;
			switch (lvl)
			{
			case DEBUG_ERROR:
				lvlText = F(INTL_ERROR);
				break;
			case DEBUG_WARNING:
				lvlText = F(INTL_WARNING);
				break;
			case DEBUG_MIN_INFO:
				lvlText = F(INTL_MIN_INFO);
				break;
			case DEBUG_MED_INFO:
				lvlText = F(INTL_MED_INFO);
				break;
			case DEBUG_MAX_INFO:
				lvlText = F(INTL_MAX_INFO);
				break;
			default:
				lvlText = F(INTL_NONE);
			}

			page_content += lvlText;
			page_content += F(".</h3>");
		}
	}

	page_content += F("<br/><pre id='slog' class='panels'>");
	page_content += Debug.popLines();
	page_content += F("</pre>");
	page_content += F("<script>"
					  "function slog_update() {"
					  "fetch('/serial').then(r => r.text()).then((r) => {"
					  "document.getElementById('slog').innerText += r;}).catch(err => console.log(err));};"
					  "setInterval(slog_update, 3000);"
					  "</script>");
	page_content += F("<h4>");
	page_content += FPSTR(INTL_DEBUG_SETTING_TO);
	page_content += F("</h4>"
					  "<table style='width:100%;'>"
					  "<tr><td style='width:25%;'><a class='b' href='/debug?lvl=0'>" INTL_NONE "</a></td>"
					  "<td style='width:25%;'><a class='b' href='/debug?lvl=1'>" INTL_ERROR "</a></td>"
					  "<td style='width:25%;'><a class='b' href='/debug?lvl=3'>" INTL_MIN_INFO "</a></td>"
					  "<td style='width:25%;'><a class='b' href='/debug?lvl=5'>" INTL_MAX_INFO "</a></td>"
					  "</tr><tr>"
					  "</tr>"
					  "</table>");

	end_html_page(page_content);
}

/*****************************************************************
 * Webserver remove config                                       *
 *****************************************************************/
static void webserver_removeConfig()
{
	// For any work with SPIFFS or server, the interrupts must be deactivated. The matrix is turned off.

	if (cfg::has_matrix)
	{
		display_update_enable(false);
	}

	if (!webserver_request_auth())
	{
		return;
	}

	RESERVE_STRING(page_content, LARGE_STR);
	start_html_page(page_content, FPSTR(INTL_DELETE_CONFIG));
	debug_outln_info(F("ws: removeConfig ..."));

	if (server.method() == HTTP_GET)
	{
		page_content += FPSTR(WEB_REMOVE_CONFIG_CONTENT);
	}
	else
	{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		// Silently remove the desaster backup
		SPIFFS.remove(F("/config.json.old"));
		if (SPIFFS.exists(F("/config.json")))
		{ // file exists
			debug_outln_info(F("removing config.json..."));
			if (SPIFFS.remove(F("/config.json")))
			{
				page_content += F("<h3>" INTL_CONFIG_DELETED ".</h3>");
			}
			else
			{
				page_content += F("<h3>" INTL_CONFIG_CAN_NOT_BE_DELETED ".</h3>");
			}
		}
		else
		{
			page_content += F("<h3>" INTL_CONFIG_NOT_FOUND ".</h3>");
		}
#pragma GCC diagnostic pop
	}
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver reset NodeMCU                                       *
 *****************************************************************/
static void webserver_reset()
{
	// For any work with SPIFFS or server, the interrupts must be deactivated. The matrix is turned off.

	if (cfg::has_matrix)
	{
		display_update_enable(false);
	}

	if (!webserver_request_auth())
	{
		return;
	}

	String page_content;
	page_content.reserve(512);

	start_html_page(page_content, FPSTR(INTL_RESTART_SENSOR));
	debug_outln_info(F("ws: reset ..."));

	if (server.method() == HTTP_GET)
	{
		page_content += FPSTR(WEB_RESET_CONTENT);
	}
	else
	{

		sensor_restart();
	}
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver data.json                                           *
 *****************************************************************/
static void webserver_data_json()
{
	String s1;
	unsigned long age = 0;

	debug_outln_info(F("ws: data json..."));
	if (!count_sends)
	{
		s1 = FPSTR(data_first_part);
		s1 += "]}";
		age = cfg::sending_intervall_ms - msSince(starttime);
		if (age > cfg::sending_intervall_ms)
		{
			age = 0;
		}
		age = 0 - age;
	}
	else
	{
		s1 = last_data_string;
		age = msSince(starttime);
		if (age > cfg::sending_intervall_ms)
		{
			age = 0;
		}
	}
	String s2 = F(", \"age\":\"");
	s2 += String((long)((age + 500) / 1000));
	s2 += F("\", \"sensordatavalues\"");
	s1.replace(F(", \"sensordatavalues\""), s2);
	server.send(200, FPSTR(TXT_CONTENT_TYPE_JSON), s1);
}

/*****************************************************************
 * Webserver metrics endpoint                                    *
 *****************************************************************/
static void webserver_metrics_endpoint()
{
	debug_outln_info(F("ws: /metrics"));
	RESERVE_STRING(page_content, XLARGE_STR);
	page_content = F("software_version{version=\"" SOFTWARE_VERSION_STR "\",$i} 1\nuptime_ms{$i} $u\nsending_intervall_ms{$i} $s\nnumber_of_measurements{$i} $c\n");
	String id(F("node=\"" SENSOR_BASENAME));
	//String id(F("node=\"" HOSTNAME_BASE));
	id += esp_chipid;
	id += '\"';
	page_content.replace("$i", id);
	page_content.replace("$u", String(msSince(time_point_device_start_ms)));
	page_content.replace("$s", String(cfg::sending_intervall_ms));
	page_content.replace("$c", String(count_sends));
	DynamicJsonDocument json2data(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json2data, last_data_string);
	if (!err)
	{
		for (JsonObject measurement : json2data[FPSTR(JSON_SENSOR_DATA_VALUES)].as<JsonArray>())
		{
			page_content += measurement["value_type"].as<const char *>();
			page_content += '{';
			page_content += id;
			page_content += "} ";
			page_content += measurement["value"].as<const char *>();
			page_content += '\n';
		}
		page_content += F("last_sample_age_ms{");
		page_content += id;
		page_content += "} ";
		page_content += String(msSince(starttime));
		page_content += '\n';
	}
	else
	{
		debug_outln_error(FPSTR(DBG_TXT_DATA_READ_FAILED));
	}
	page_content += F("# EOF\n");
	debug_outln(page_content, DEBUG_MED_INFO);
	server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), page_content);
}

/*****************************************************************
 * Webserver Images                                              *
 *****************************************************************/

static void webserver_favicon()
{
	server.sendHeader(F("Cache-Control"), F("max-age=2592000, public"));

	server.send_P(200, TXT_CONTENT_TYPE_IMAGE_PNG,
				  AIRCARTO_INFO_LOGO_PNG, AIRCARTO_INFO_LOGO_PNG_SIZE);
}

/*****************************************************************
 * Webserver page not found                                      *
 *****************************************************************/
static void webserver_not_found()
{

	last_page_load = millis();
	debug_outln_info(F("ws: not found ..."));

	if (WiFi.status() != WL_CONNECTED)
	{
		if ((server.uri().indexOf(F("success.html")) != -1) || (server.uri().indexOf(F("detect.html")) != -1))
		{
			server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), FPSTR(WEB_IOS_REDIRECT));
		}
		else
		{
			sendHttpRedirect();
		}
	}
	else
	{
		server.send(404, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), F("Not found."));
	}
}

static void webserver_static()
{
	server.sendHeader(F("Cache-Control"), F("max-age=2592000, public"));

	if (server.arg(String('r')) == F("logo"))
	{

		server.send_P(200, TXT_CONTENT_TYPE_IMAGE_PNG,
					  AIRCARTO_INFO_LOGO_PNG, AIRCARTO_INFO_LOGO_PNG_SIZE);
	}
	else if (server.arg(String('r')) == F("css"))
	{
		server.send_P(200, TXT_CONTENT_TYPE_TEXT_CSS,
					  WEB_PAGE_STATIC_CSS, sizeof(WEB_PAGE_STATIC_CSS) - 1);
	}
	else
	{
		webserver_not_found();
	}
}

/*****************************************************************
 * Webserver setup                                               *
 *****************************************************************/
static void setup_webserver()
{
	server.on("/", webserver_root);
	server.on(F("/config"), webserver_config);
	server.on(F("/wifi"), webserver_wifi);
	server.on(F("/values"), webserver_values);
	server.on(F("/status"), webserver_status);
	server.on(F("/generate_204"), webserver_config);
	server.on(F("/fwlink"), webserver_config);
	server.on(F("/debug"), webserver_debug_level);
	server.on(F("/serial"), webserver_serial);
	server.on(F("/removeConfig"), webserver_removeConfig);
	server.on(F("/reset"), webserver_reset);
	server.on(F("/data.json"), webserver_data_json);
	server.on(F("/metrics"), webserver_metrics_endpoint);
	server.on(F("/favicon.ico"), webserver_favicon);
	server.on(F(STATIC_PREFIX), webserver_static);
	server.onNotFound(webserver_not_found);
	debug_outln_info(F("Starting Webserver... "));
	server.begin();
}

static int selectChannelForAp()
{
	std::array<int, 14> channels_rssi;
	std::fill(channels_rssi.begin(), channels_rssi.end(), -100);

	for (unsigned i = 0; i < std::min((uint8_t)14, count_wifiInfo); i++)
	{
		if (wifiInfo[i].RSSI > channels_rssi[wifiInfo[i].channel])
		{
			channels_rssi[wifiInfo[i].channel] = wifiInfo[i].RSSI;
		}
	}

	if ((channels_rssi[1] < channels_rssi[6]) && (channels_rssi[1] < channels_rssi[11]))
	{
		return 1;
	}
	else if ((channels_rssi[6] < channels_rssi[1]) && (channels_rssi[6] < channels_rssi[11]))
	{
		return 6;
	}
	else
	{
		return 11;
	}
}

/*****************************************************************
 * WifiConfig                                                    *
 *****************************************************************/

static void wifiConfig()
{

	if (cfg::has_matrix)
	{
		display_update_enable(false);
	}

	debug_outln_info(F("Starting WiFiManager"));
	debug_outln_info(F("AP ID: "), String(cfg::fs_ssid));
	debug_outln_info(F("Password: "), String(cfg::fs_pwd));

	wificonfig_loop = true;
	WiFi.disconnect(true, true);

	debug_outln_info(F("scan for wifi networks..."));
	int8_t scanReturnCode = WiFi.scanNetworks(false /* scan async */, true /* show hidden networks */);
	if (scanReturnCode < 0)
	{
		debug_outln_error(F("WiFi scan failed. Treating as empty. "));
		count_wifiInfo = 0;
	}
	else
	{
		count_wifiInfo = (uint8_t)scanReturnCode;
	}

	delete[] wifiInfo;
	wifiInfo = new struct_wifiInfo[std::max(count_wifiInfo, (uint8_t)1)];

	for (unsigned i = 0; i < count_wifiInfo; i++)
	{
		String SSID;
		uint8_t *BSSID;

		memset(&wifiInfo[i], 0, sizeof(struct_wifiInfo));
		WiFi.getNetworkInfo(i, SSID, wifiInfo[i].encryptionType, wifiInfo[i].RSSI, BSSID, wifiInfo[i].channel);
		SSID.toCharArray(wifiInfo[i].ssid, sizeof(wifiInfo[0].ssid));
	}

	// Use 13 channels if locale is not "EN"
	wifi_country_t wifi;
	wifi.policy = WIFI_COUNTRY_POLICY_MANUAL;
	strcpy(wifi.cc, INTL_LANG);
	wifi.nchan = (INTL_LANG[0] == 'E' && INTL_LANG[1] == 'N') ? 11 : 13;
	wifi.schan = 1;

	WiFi.mode(WIFI_AP);
	const IPAddress apIP(192, 168, 4, 1);
	WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
	WiFi.softAP(cfg::fs_ssid, cfg::fs_pwd, selectChannelForAp());
	// In case we create a unique password at first start
	debug_outln_info(F("AP Password is: "), cfg::fs_pwd);

	DNSServer dnsServer;
	// Ensure we don't poison the client DNS cache
	dnsServer.setTTL(0);
	dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
	dnsServer.start(53, "*", apIP); // 53 is port for DNS server

	setup_webserver();

	// 10 minutes timeout for wifi config
	last_page_load = millis();

	while ((millis() - last_page_load) < cfg::time_for_wifi_config + 500)
	{
		dnsServer.processNextRequest();
		server.handleClient();
		yield();
	}

	WiFi.softAPdisconnect(true);
	dnsServer.stop(); // A VOIR
	delay(100);
	// WiFi.disconnect(true, true);
	WiFi.mode(WIFI_OFF); //A tenter

	debug_outln_info(F("---- Result Webconfig ----"));
	debug_outln_info(F("WiFi: "), cfg::has_wifi);
	debug_outln_info(F("WLANSSID: "), cfg::wlanssid);
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_info_bool(F("MHZ16: "), cfg::mhz16_read);
	debug_outln_info_bool(F("MHZ19: "), cfg::mhz19_read);
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_info_bool(F("CSV: "), cfg::send2csv);
	debug_outln_info_bool(F("AirCarto: "), cfg::send2custom);
	debug_outln_info_bool(F("Custom API 2: "), cfg::send2custom2);
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_info_bool(F("Matrix: "), cfg::has_matrix);
	debug_outln_info(F("Debug: "), String(cfg::debug));
	wificonfig_loop = false; // VOIR ICI
}

static void waitForWifiToConnect(int maxRetries)
{
	int retryCount = 0;
	while ((WiFi.status() != WL_CONNECTED) && (retryCount < maxRetries))
	{
		delay(500);
		debug_out(".", DEBUG_MIN_INFO);
		++retryCount;
	}
}

/*****************************************************************
 * WiFi auto connecting script                                   *
 *****************************************************************/

static WiFiEventId_t disconnectEventHandler;
static WiFiEventId_t connectEventHandler;
static WiFiEventId_t STAstartEventHandler;
static WiFiEventId_t STAstopEventHandler;

static void connectWifi()
{
	if (cfg::has_matrix)
	{
		display_update_enable(false);
	}

	disconnectEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
										  {
											  if (!wifi_connection_lost)
											  {
												  Debug.println("Event disconnect");
												  wifi_connection_lost = true;
											  }
										  },
										  WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

	connectEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
									   {
										   if (wifi_connection_lost)
										   {
											   Debug.println("Event connect");
											   wifi_connection_lost = false;
										   }
									   },
									   WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);

	STAstartEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
										{ Debug.println("STA start"); },
										WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_START);

	STAstopEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
									   { Debug.println("STA stop"); },
									   WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_STOP);

	if (WiFi.getAutoConnect())
	{
		WiFi.setAutoConnect(false);
	}

	WiFi.setAutoReconnect(false);

	// Use 13 channels for connect to known AP
	wifi_country_t wifi;
	wifi.policy = WIFI_COUNTRY_POLICY_MANUAL;
	strcpy(wifi.cc, INTL_LANG);
	wifi.nchan = 13;
	wifi.schan = 1;

	WiFi.mode(WIFI_STA);
	WiFi.setHostname(cfg::fs_ssid);
	WiFi.begin(cfg::wlanssid, cfg::wlanpwd); // Start WiFI

	debug_outln_info(FPSTR(DBG_TXT_CONNECTING_TO), cfg::wlanssid);

	waitForWifiToConnect(20); //diminur ici ???  //40
	debug_outln_info(emptyString);

	//if (WiFi.status() != WL_CONNECTED) //Waitforwifistatus ?
	if (WiFi.waitForConnectResult(10000) != WL_CONNECTED) //Waitforwifistatus ?
	{
		Debug.println("Force change WiFi config");
		wifi_connection_lost = true;
		cfg::has_wifi = false;
		// strcpy_P(cfg::wlanssid, "TYPE SSID");
		// strcpy_P(cfg::wlanpwd, "TYPE PWD");

		if (cfg::has_matrix)
		{
			display_update_enable(true);
		}
		wifiConfig();
	}
	else
	{
		wifi_connection_lost = false;
	}

	debug_outln_info(F("WiFi connected, IP is: "), WiFi.localIP().toString());
	last_signal_strength = WiFi.RSSI(); //RSSI ICI!!!!

	if (MDNS.begin(cfg::fs_ssid))
	{
		MDNS.addService("http", "tcp", 80);
		MDNS.addServiceTxt("http", "tcp", "PATH", "/config");
	}

	if (cfg::has_matrix)
	{
		display_update_enable(true); //reactivate matrix
	}
}

// static void reConnectWifi()
// {
// 	if (cfg::has_matrix)
// 	{
// 		display_update_enable(false);
// 	}

// 	display_debug(F("Connecting to"), String(cfg::wlanssid));

// 	disconnectEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
// 										  {
// 											  if (!wifi_connection_lost){
// 												Debug.println("Event disconnect");
// 											  wifi_connection_lost = true;
// 											  }
// 										  },
// 										  WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

// 	connectEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
// 										  {
// 											if (wifi_connection_lost){
// 												Debug.println("Event connect");
// 											 wifi_connection_lost = false;
// 											}
// 										  },
// 										  WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);

// 	STAstartEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
// 										  {
// 											Debug.println("STA start");
// 										  },
// 										  WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_START);

// 	STAstopEventHandler = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
// 										  {
// 											Debug.println("STA stop");
// 										  },
// 										  WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_STOP);

// 	if (WiFi.getAutoConnect())
// 	{
// 		WiFi.setAutoConnect(false);
// 	}

// 	WiFi.setAutoReconnect(false);

// 	// Use 13 channels for connect to known AP
// 	wifi_country_t wifi;
// 	wifi.policy = WIFI_COUNTRY_POLICY_MANUAL;
// 	strcpy(wifi.cc, INTL_LANG);
// 	wifi.nchan = 13;
// 	wifi.schan = 1;

// 	WiFi.mode(WIFI_STA);
// 	WiFi.setHostname(cfg::fs_ssid);
// 	WiFi.begin(cfg::wlanssid, cfg::wlanpwd); // Start WiFI

// 	debug_outln_info(FPSTR(DBG_TXT_CONNECTING_TO), cfg::wlanssid);

// 	waitForWifiToConnect(40);
// 	debug_outln_info(emptyString);

// 	if (WiFi.waitForConnectResult(10000) != WL_CONNECTED)
// 	{
// 		Debug.println("Can't restart!");
// 		// sensor_restart();
// 		wifi_connection_lost = true;
// 	}
// 	else
// 	{
// 		wifi_connection_lost = false;
// 	}

// 	debug_outln_info(F("WiFi connected, IP is: "), WiFi.localIP().toString());
// 	last_signal_strength = WiFi.RSSI();

// 	if (MDNS.begin(cfg::fs_ssid))
// 	{
// 		MDNS.addService("http", "tcp", 80);
// 		MDNS.addServiceTxt("http", "tcp", "PATH", "/config");
// 	}

// 	if (cfg::has_matrix)
// 	{
// 		display_update_enable(true); //reactivate matrix
// 	}
// }

static WiFiClient *getNewLoggerWiFiClient(const LoggerEntry logger)
{
	WiFiClient *_client;
	_client = new WiFiClient;
	return _client;
}

static WiFiClientSecure *getNewLoggerWiFiClientSecure(const LoggerEntry logger)
{
	WiFiClientSecure *_client;
	_client = new WiFiClientSecure;
	return _client;
}

/*****************************************************************
 * send data to rest api                                         *
 *****************************************************************/
static unsigned long sendData(const LoggerEntry logger, const String &data, const int pin, const char *host, const char *url, bool ssl)
{
	unsigned long start_send = millis();
	const __FlashStringHelper *contentType;
	int result = 0;

	String s_Host(FPSTR(host));
	String s_url(FPSTR(url));

	switch (logger)
	{
	case LoggerCustom:
		Debug.print("LoggerAirCarto https: ");
		Debug.println(ssl);
		contentType = FPSTR(TXT_CONTENT_TYPE_JSON);
		break;
	case LoggerCustom2:
		Debug.print("LoggerAtmoSud https: ");
		Debug.println(ssl);
		contentType = FPSTR(TXT_CONTENT_TYPE_JSON);
		break;
	default:
		contentType = FPSTR(TXT_CONTENT_TYPE_JSON);
		break;
	}

	if (!ssl)
	{
		std::unique_ptr<WiFiClient> client(getNewLoggerWiFiClient(logger));

		HTTPClient http;
		http.setTimeout(20 * 1000);
		http.setUserAgent(SOFTWARE_VERSION + '/' + esp_chipid);
		http.setReuse(false);
		bool send_success = false;

		if (logger == LoggerCustom && (*cfg::user_custom || *cfg::pwd_custom))
		{
			http.setAuthorization(cfg::user_custom, cfg::pwd_custom);
		}

		if (logger == LoggerCustom2 && (*cfg::user_custom2 || *cfg::pwd_custom2))
		{
			http.setAuthorization(cfg::user_custom2, cfg::pwd_custom2);
		}

		if (http.begin(*client, s_Host, loggerConfigs[logger].destport, s_url, !!loggerConfigs[logger].session))
		{
			http.addHeader(F("Content-Type"), contentType);
			http.addHeader(F("X-Sensor"), String(F(SENSOR_BASENAME)) + esp_chipid);
			// http.addHeader(F("X-MAC-ID"), String(F(SENSOR_BASENAME)) + esp_mac_id);
			if (pin)
			{
				http.addHeader(F("X-PIN"), String(pin));
			}

			result = http.POST(data);

			if (result >= HTTP_CODE_OK && result <= HTTP_CODE_ALREADY_REPORTED)
			{
				debug_outln_info(F("Succeeded http - "), s_Host);
				send_success = true;
			}
			else if (result >= HTTP_CODE_BAD_REQUEST)
			{
				debug_outln_info(F("Request http failed with error: "), String(result));
				debug_outln_info(F("Details:"), http.getString());
			}
			http.end();
		}
		else
		{
			debug_outln_info(F("Failed connecting to "), s_Host);
		}
		if (!send_success && result != 0)
		{
			loggerConfigs[logger].errors++;
			last_sendData_returncode = result;
		}

		return millis() - start_send;
	}
	else
	{
		std::unique_ptr<WiFiClientSecure> clientSecure(getNewLoggerWiFiClientSecure(logger));

		switch (logger)
		{
		case LoggerCustom:
			clientSecure->setCACert(ca_aircarto);
			break;
		case LoggerCustom2:
			clientSecure->setCACert(ca_atmo);
			break;
		}

		HTTPClient https;
		https.setTimeout(20 * 1000);
		https.setUserAgent(SOFTWARE_VERSION + '/' + esp_chipid);
		https.setReuse(false);
		bool send_success = false;
		if (logger == LoggerCustom && (*cfg::user_custom || *cfg::pwd_custom))
		{
			https.setAuthorization(cfg::user_custom, cfg::pwd_custom);
		}
		if (logger == LoggerCustom2 && (*cfg::user_custom2 || *cfg::pwd_custom2))
		{
			https.setAuthorization(cfg::user_custom2, cfg::pwd_custom2);
		}

		if (https.begin(*clientSecure, s_Host, loggerConfigs[logger].destport, s_url, !!loggerConfigs[logger].session))
		{
			https.addHeader(F("Content-Type"), contentType);
			https.addHeader(F("X-Sensor"), String(F(SENSOR_BASENAME)) + esp_chipid);
			// https.addHeader(F("X-MAC-ID"), String(F(SENSOR_BASENAME)) + esp_mac_id);
			if (pin)
			{
				https.addHeader(F("X-PIN"), String(pin));
			}

			result = https.POST(data);

			if (result >= HTTP_CODE_OK && result <= HTTP_CODE_ALREADY_REPORTED)
			{
				debug_outln_info(F("Succeeded https - "), s_Host);
				send_success = true;
			}
			else if (result >= HTTP_CODE_BAD_REQUEST)
			{
				debug_outln_info(F("Request https failed with error: "), String(result));
				debug_outln_info(F("Details:"), https.getString());
			}
			https.end();
		}
		else
		{
			debug_outln_info(F("Failed connecting to "), s_Host);
		}
		if (!send_success && result != 0)
		{
			loggerConfigs[logger].errors++;
			last_sendData_returncode = result;
		}

		return millis() - start_send;
	}
}

/*****************************************************************
 * send data as csv to serial out                                *
 *****************************************************************/
static void send_csv(const String &data)
{
	DynamicJsonDocument json2data(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json2data, data);
	debug_outln_info(F("CSV Output: "), data);
	if (!err)
	{
		String headline = F("Timestamp_ms;");
		String valueline(act_milli);
		valueline += ';';
		for (JsonObject measurement : json2data[FPSTR(JSON_SENSOR_DATA_VALUES)].as<JsonArray>())
		{
			headline += measurement["value_type"].as<const char *>();
			headline += ';';
			valueline += measurement["value"].as<const char *>();
			valueline += ';';
		}
		static bool first_csv_line = true;
		if (first_csv_line)
		{
			if (headline.length() > 0)
			{
				headline.remove(headline.length() - 1);
			}
			Debug.println(headline);
			first_csv_line = false;
		}
		if (valueline.length() > 0)
		{
			valueline.remove(valueline.length() - 1);
		}
		Debug.println(valueline);
	}
	else
	{
		debug_outln_error(FPSTR(DBG_TXT_DATA_READ_FAILED));
	}
}

/*****************************************************************
 * read MHZ16 sensor values                              *
 *****************************************************************/
static void fetchSensorMHZ16(String &s)
{
	const char *const sensor_name = SENSORS_MHZ16;
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(sensor_name));

	int value = mhz16.getPPM();

	if (isnan(value))
	{
		debug_outln_error(F("MHZ16 read failed"));
	}
	else
	{
		int value = mhz16.getPPM();
		mhz16_sum += value;
		mhz16_val_count++;
		debug_outln(String(mhz16_val_count), DEBUG_MAX_INFO);
	}

	if (send_now && cfg::sending_intervall_ms >= 120000)
	{
		last_value_MHZ16 = -1.0f;

		if (mhz16_val_count == 12)
		{
			last_value_MHZ16 = float(mhz16_sum / mhz16_val_count);
			add_Value2Json(s, F("MHZ16_CO2"), FPSTR(DBG_TXT_CO2PPM), last_value_MHZ16);
			debug_outln_info(FPSTR(DBG_TXT_SEP));
		}
		else
		{
			MHZ16_error_count++;
		}

		mhz16_sum = 0;
		mhz16_val_count = 0;
	}

	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(sensor_name));
}

/*****************************************************************
 * read MHZ19 sensor values                              *
 *****************************************************************/
static void fetchSensorMHZ19(String &s)
{
	const char *const sensor_name = SENSORS_MHZ19;
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(sensor_name));

	int value;

	value = mhz19.getCO2();

	if (isnan(value))
	{
		debug_outln_error(F("MHZ19 read failed"));
	}
	else
	{
		mhz19_sum += value;
		mhz19_val_count++;
		debug_outln(String(mhz19_val_count), DEBUG_MAX_INFO);
	}

	if (send_now && cfg::sending_intervall_ms == 120000)
	{
		last_value_MHZ19 = -1.0f;

		if (mhz19_val_count >= 12)
		{
			last_value_MHZ19 = float(mhz19_sum / mhz19_val_count);
			add_Value2Json(s, F("MHZ19_CO2"), FPSTR(DBG_TXT_CO2PPM), last_value_MHZ19);
			debug_outln_info(FPSTR(DBG_TXT_SEP));
		}
		else
		{
			MHZ19_error_count++;
		}

		mhz19_sum = 0;
		mhz19_val_count = 0;
	}

	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(sensor_name));
}

static void display_values_matrix()
{
	float co2_value = -1.0;

	if (cfg::mhz16_read)
	{
		co2_value = last_value_MHZ16;
	}

	if (cfg::mhz19_read)
	{
		co2_value = last_value_MHZ19;
	}

	if (co2_value == -1.0 && count_sends == 0)
	{
		//ECRAN ATTENTE+
		// on remplit array avec uint16_t
		// drawRandomPixel(0, 0, 64, 64);

		int div_entiere = (millis() - time_animation) / LEN_ANIMATION;
		switch (div_entiere)
		{
		case 0:
			drawImage(0, 0, 64, 64, wait0);
			break;
		case 1 ... 67:
			drawImage(0, 0, 64, 64, wait1);
			break;
		case 68 ... 71:
			drawImage(0, 0, 64, 64, wait2);
			break;
		case 72 ... 75:
			drawImage(0, 0, 64, 64, wait3);
			break;
		case 76 ... 79:
			drawImage(0, 0, 64, 64, wait4);
			break;
		case 80 ... 83:
			drawImage(0, 0, 64, 64, wait5);
			break;
		case 84 ... 87:
			drawImage(0, 0, 64, 64, wait6);
			break;
		case 88 ... 154:
			drawImage(0, 0, 64, 64, wait7);
			break;
		default:
			time_animation = millis();
			break;
		}
	}

	if (co2_value != -1.0 && count_sends > 0)
	{
		int div_entiere = (millis() - time_animation) / LEN_ANIMATION;

		if (co2_value <= 800)
		{

			switch (div_entiere)
			{
			case 0:
				drawImage(0, 0, 64, 64, anim_1_0);
				break;
			case 1 ... 67:
				drawImage(0, 0, 64, 64, anim_1_1);
				break;
			case 68 ... 71:
				drawImage(0, 0, 64, 64, anim_1_2);
				break;
			case 72 ... 75:
				drawImage(0, 0, 64, 64, anim_1_3);
				break;
			case 76 ... 79:
				drawImage(0, 0, 64, 64, anim_1_4);
				break;
			case 80 ... 83:
				drawImage(0, 0, 64, 64, anim_1_5);
				break;
			case 84 ... 87:
				drawImage(0, 0, 64, 64, anim_1_6);
				break;
			case 88 ... 154:
				drawImage(0, 0, 64, 64, anim_1_7);
				break;
			default:
				time_animation = millis();
			}
		}

		if (co2_value > 800 && co2_value <= 1500)
		{

			switch (div_entiere)
			{
			case 0:
				drawImage(0, 0, 64, 64, anim_2_0);
				break;
			case 1 ... 67:
				drawImage(0, 0, 64, 64, anim_2_1);
				break;
			case 68 ... 71:
				drawImage(0, 0, 64, 64, anim_2_2);
				break;
			case 72 ... 75:
				drawImage(0, 0, 64, 64, anim_2_3);
				break;
			case 76 ... 79:
				drawImage(0, 0, 64, 64, anim_2_4);
				break;
			case 80 ... 83:
				drawImage(0, 0, 64, 64, anim_2_5);
				break;
			case 84 ... 87:
				drawImage(0, 0, 64, 64, anim_2_6);
				break;
			case 88 ... 154:
				drawImage(0, 0, 64, 64, anim_2_7);
				break;
			default:
				time_animation = millis();
			}
		}

		if (co2_value > 1500)
		{

			switch (div_entiere)
			{
			case 0:
				drawImage(0, 0, 64, 64, anim_3_0);
				break;
			case 1 ... 67:
				drawImage(0, 0, 64, 64, anim_3_1);
				break;
			case 68 ... 71:
				drawImage(0, 0, 64, 64, anim_3_2);
				break;
			case 72 ... 75:
				drawImage(0, 0, 64, 64, anim_3_3);
				break;
			case 76 ... 79:
				drawImage(0, 0, 64, 64, anim_3_4);
				break;
			case 80 ... 83:
				drawImage(0, 0, 64, 64, anim_3_5);
				break;
			case 84 ... 87:
				drawImage(0, 0, 64, 64, anim_3_6);
				break;
			case 88 ... 154:
				drawImage(0, 0, 64, 64, anim_3_7);
				break;
			default:
				time_animation = millis();
			}
		}
	}

	if (co2_value == -1.0 && count_sends > 0)
	{
		//erreur
		display.fillRect(0, 0, 64, 64, display.color565(0, 0, 0));
	}

	yield();
}

/*****************************************************************
 * Init matrix                                         *
 *****************************************************************/

static void init_matrix()
{
	timer = timerBegin(0, 80, true); //init timer once only
	//display.begin(16);
	display.begin(32);
	display.setDriverChip(SHIFT);  // SHIFT ou FM6124 ou FM6126A
	display.setColorOrder(RRGGBB); // ATTENTION à changer en fonction de l'écran !!!! Small Matrix (160x80mm) is RRBBGG and Big Matrix (192x96mm) is RRGGBB
	display_update_enable(true);
	display.setFont(NULL); //Default font
}

/*****************************************************************
   Functions
 *****************************************************************/

static void logEnabledAPIs()
{
	if (cfg::send2csv)
	{
		debug_outln_info(F("Serial as CSV"));
	}

	if (cfg::send2custom)
	{
		debug_outln_info(F("AirCarto API"));
	}
	if (cfg::send2custom2)
	{
		debug_outln_info(F("Atmosud API"));
	}
}

static void setupNetworkTime()
{
	// server name ptrs must be persisted after the call to configTime because internally
	// the pointers are stored see implementation of lwip sntp_setservername()
	static char ntpServer1[18], ntpServer2[18];
	strcpy_P(ntpServer1, NTP_SERVER_1);
	strcpy_P(ntpServer2, NTP_SERVER_2);
	configTime(0, 0, ntpServer1, ntpServer2);
}

static unsigned long sendDataToOptionalApis(const String &data)
{
	unsigned long sum_send_time = 0;

	Debug.println(data);

	if (cfg::send2custom)
	{
		String data_to_send = data;
		data_to_send.remove(0, 1);
		String data_4_custom(F("{\"moduleairid\": \""));
		data_4_custom += esp_chipid;
		data_4_custom += "\", ";
		data_4_custom += data_to_send;
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("aircarto api: "));
		sum_send_time += sendData(LoggerCustom, data_4_custom, 0, cfg::host_custom, cfg::url_custom, cfg::ssl_custom);
	}

	if (cfg::send2custom2)
	{
		String data_to_send = data;
		data_to_send.remove(0, 1);
		String data_4_custom(F("{\"moduleairid\": \""));
		data_4_custom += esp_chipid;
		data_4_custom += "\", ";
		data_4_custom += data_to_send;
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("atmosud api: "));
		sum_send_time += sendData(LoggerCustom2, data_4_custom, 0, cfg::host_custom2, cfg::url_custom2, cfg::ssl_custom2);
	}

	if (cfg::send2csv)
	{
		debug_outln_info(F("## Sending as csv: "));
		send_csv(data);
	}

	return sum_send_time;
}

/*****************************************************************
 * Check stack                                                    *
 *****************************************************************/
void *StackPtrAtStart;
void *StackPtrEnd;
UBaseType_t watermarkStart;

/*****************************************************************
 * The Setup                                                     *
 *****************************************************************/

void setup()
{
	void *SpStart = NULL;
	StackPtrAtStart = (void *)&SpStart;
	watermarkStart = uxTaskGetStackHighWaterMark(NULL);
	StackPtrEnd = StackPtrAtStart - watermarkStart;

	Debug.begin(115200); // Output to Serial at 115200 baud
	Debug.println(F("Starting"));

	Debug.printf("\r\n\r\nAddress of Stackpointer near start is:  %p \r\n", (void *)StackPtrAtStart);
	Debug.printf("End of Stack is near: %p \r\n", (void *)StackPtrEnd);
	Debug.printf("Free Stack at start of setup() is:  %d \r\n", (uint32_t)StackPtrAtStart - (uint32_t)StackPtrEnd);

	Debug.printf("ESP32 Partition table:\n\n");
	Debug.printf("| Type | Sub |  Offset  |   Size   |       Label      |\n");
	Debug.printf("| ---- | --- | -------- | -------- | ---------------- |\n");
	esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
	if (pi != NULL)
	{
		do
		{
			const esp_partition_t *p = esp_partition_get(pi);
			Debug.printf("|  %02x  | %02x  | 0x%06X | 0x%06X | %-16s |\r\n", p->type, p->subtype, p->address, p->size, p->label);
		} while (pi = (esp_partition_next(pi)));
	}

	esp_chipid = String((uint16_t)(ESP.getEfuseMac() >> 32), HEX); // for esp32
	esp_chipid += String((uint32_t)ESP.getEfuseMac(), HEX);
	esp_chipid.toUpperCase();
	cfg::initNonTrivials(esp_chipid.c_str());
	WiFi.persistent(false);

	debug_outln_info(F("ModuleAir_Art: " SOFTWARE_VERSION_STR "/"), String(CURRENT_LANG));

	init_config();
	Debug.print("Actual SPIFFS size: ");
	Debug.println(SPIFFS.usedBytes());

	spiffs_matrix = cfg::has_matrix; //save the spiffs state on start

	Debug.println("spiffs_matrix: ");
	Debug.println(spiffs_matrix);

	if (cfg::has_matrix)
	{
		init_matrix();
	}

	if (cfg::mhz16_read || cfg::mhz19_read)
	{
		//serialMHZ.begin(9600, SERIAL_8N1, CO2_SERIAL_RX, CO2_SERIAL_TX);
		Debug.println("serialMHZ 9600 8N1");
		//serialMHZ.setTimeout((4 * 12 * 1000) / 9600); //VOIR ICI LE TIMEOUT

		if (cfg::mhz16_read)
		{
			mhz16.begin(CO2_SERIAL_RX, CO2_SERIAL_TX, 2);
			//mhz16.autoCalibration(false);
		}

		if (cfg::mhz19_read)
		{
			serialMHZ.begin(9600, SERIAL_8N1, CO2_SERIAL_RX, CO2_SERIAL_TX);
			mhz19.begin(serialMHZ);
			//mhz19.autoCalibration(false);
		}
	}

	debug_outln_info(F("\nChipId: "), esp_chipid);

	if (cfg::has_wifi)
	{
		setupNetworkTime();
		connectWifi();
		setup_webserver();
	}
	else
	{
		wifiConfig();
	}

	createLoggerConfigs();
	logEnabledAPIs();

	delay(50);

	starttime = millis(); // store the start time
	last_update_attempt = time_point_device_start_ms = starttime;

	if (cfg::mhz16_read)
	{
		last_display_millis_oled = starttime_MHZ16 = starttime;
		last_display_millis_matrix = starttime_MHZ16 = starttime;
	}

	if (cfg::mhz19_read)
	{
		last_display_millis_oled = starttime_MHZ19 = starttime;
		last_display_millis_matrix = starttime_MHZ19 = starttime;
	}

	Debug.printf("End of void setup()\n");
	time_end_setup = millis();
}

void loop()
{
	String result_MHZ16, result_MHZ19;

	unsigned sum_send_time = 0;

	act_micro = micros();
	act_milli = millis();
	send_now = msSince(starttime) > cfg::sending_intervall_ms;

	// Wait at least 30s for each NTP server to sync

	if (cfg::has_wifi && !wifi_connection_lost)
	{
		if (!sntp_time_set && send_now && msSince(time_point_device_start_ms) < 1000 * 2 * 30 + 5000)
		{
			debug_outln_info(F("NTP sync not finished yet, skipping send"));
			send_now = false;
			starttime = act_milli;
		}
	}

	sample_count++;

	if (last_micro != 0)
	{
		unsigned long diff_micro = act_micro - last_micro;
		UPDATE_MIN_MAX(min_micro, max_micro, diff_micro);
	}
	last_micro = act_micro;

	if (cfg::mhz16_read)
	{
		if ((msSince(starttime_MHZ16) > SAMPLETIME_MHZ16_MS && mhz16_val_count < 11) || send_now)
		{
			starttime_MHZ16 = act_milli;
			fetchSensorMHZ16(result_MHZ16);
		}
	}

	if (cfg::mhz19_read)
	{
		if ((msSince(starttime_MHZ19) > SAMPLETIME_MHZ19_MS && mhz19_val_count < 11) || send_now)
		{
			starttime_MHZ19 = act_milli;
			fetchSensorMHZ19(result_MHZ19);
		}
	}

	if (cfg::has_matrix)
	// if ((msSince(last_display_millis_matrix) > DISPLAY_UPDATE_INTERVAL_MS) && (cfg::has_matrix))
	{
		display_values_matrix();
		last_display_millis_matrix = act_milli;
	}

	//if (cfg::has_wifi && WiFi.waitForConnectResult(10000) == WL_CONNECTED)
	//if (cfg::has_wifi && !wifi_connection_lost)
	if (cfg::has_wifi && WiFi.status() == WL_CONNECTED)
	{
		server.handleClient();
		yield();
	}

	if (send_now && cfg::sending_intervall_ms >= 120000)
	{

		void *SpActual = NULL;
		Debug.printf("Free Stack at send_now is: %d \r\n", (uint32_t)&SpActual - (uint32_t)StackPtrEnd);

		if (cfg::has_wifi && !wifi_connection_lost)
		{
			last_signal_strength = WiFi.RSSI();
		}

		RESERVE_STRING(data, LARGE_STR);
		data = FPSTR(data_first_part);
		RESERVE_STRING(result, MED_STR);

		//These values are not sent because not configured in the SC API:

		if (cfg::mhz16_read)
		{
			data += result_MHZ16;
		}

		if (cfg::mhz19_read)
		{
			data += result_MHZ19;
		}

		add_Value2Json(data, F("samples"), String(sample_count));
		add_Value2Json(data, F("min_micro"), String(min_micro));
		add_Value2Json(data, F("max_micro"), String(max_micro));
		add_Value2Json(data, F("interval"), String(cfg::sending_intervall_ms));
		add_Value2Json(data, F("signal"), String(last_signal_strength));

		if ((unsigned)(data.lastIndexOf(',') + 1) == data.length())
		{
			data.remove(data.length() - 1);
		}
		data += "]}";

		yield();

		if (cfg::has_wifi && !wifi_connection_lost)
		{
			sum_send_time += sendDataToOptionalApis(data);
			sending_time = (3 * sending_time + sum_send_time) / 4;

			if (sum_send_time > 0)
			{
				debug_outln_info(F("Time for Sending (ms): "), String(sending_time));
			}

			//RECONNECT ETAIT ICI
		}

		if ((WiFi.status() != WL_CONNECTED || sending_time > 30000 || wifi_connection_lost) && cfg::has_wifi)
		{
			debug_outln_info(F("Connection lost, reconnecting "));
			WiFi_error_count++;
			WiFi.reconnect();
			waitForWifiToConnect(20);

			if (wifi_connection_lost && WiFi.waitForConnectResult(10000) != WL_CONNECTED)
			{
				if (cfg::has_matrix)
				{
					display_update_enable(false);
				}

				Debug.println("Reconnect failed after WiFi.reconnect()");

				WiFi.disconnect(true, true);
				// wifi_country_t wifi;
				// wifi.policy = WIFI_COUNTRY_POLICY_MANUAL;
				// strcpy(wifi.cc, INTL_LANG);
				// wifi.nchan = 13;
				// wifi.schan = 1;
				WiFi.mode(WIFI_STA);
				WiFi.setHostname(cfg::fs_ssid);
				WiFi.begin(cfg::wlanssid, cfg::wlanpwd); // Start WiFI again

				// if (MDNS.begin(cfg::fs_ssid))
				// {
				// 	MDNS.addService("http", "tcp", 80);
				// 	MDNS.addServiceTxt("http", "tcp", "PATH", "/config");
				// }

				//reConnectWifi();

				if (cfg::has_matrix)
				{
					display_update_enable(true);
				}
			}
			debug_outln_info(emptyString);
		}

		// only do a restart after finishing sending (Wifi). Befor Lora to avoid conflicts with the LMIC
		if (msSince(time_point_device_start_ms) > DURATION_BEFORE_FORCED_RESTART_MS)
		{
			sensor_restart();
		}

		// Resetting for next sampling
		last_data_string = std::move(data);
		sample_count = 0;
		last_micro = 0;
		min_micro = 1000000000;
		max_micro = 0;
		sum_send_time = 0;

		starttime = millis(); // store the start time
		count_sends++;
		time_animation = millis();
	}

	if (sample_count % 500 == 0)
	{
		//		Serial.println(ESP.getFreeHeap(),DEC);
	}
}
