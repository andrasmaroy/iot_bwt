#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#include <driver/rtc_io.h>
#include <esp32/ulp.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <ulptool.h>

#include "ulp_main.h"

// WiFi
#define WIFI_SSID ""
#define WIFI_PSK  ""

// MQTT
#define MQTT_URL      ""
#define MQTT_PORT     1883
#define MQTT_DEV      ""
#define MQTT_USER     ""
#define MQTT_PASSWORD ""

// Battery
#define BATTERY_MIN 1740
#define BATTERY_MAX 2320

// Flow sensor
#define FLOW_PULSE_PER_LITER 400
#define PIN_FLOW_SENSOR      GPIO_NUM_25
#define PIN_RTC              6  // RTC Pin number corresponding to GPIO 25

#define MAX_RETRY_COUNT 5
#define RETRY_DELAY 5000

#define WAKEUP_PERIOD 86400000000 // 1 day in us

const String HASS_TOPIC_BASE = "";

const char* Lets_Encrypt_R3 = \
    "-----BEGIN CERTIFICATE-----\n" \
    "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\n" \
    "nLRbwHOoq7hHwg==\n" \
    "-----END CERTIFICATE-----\n";

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);
static void update_pulse_count(void);

WiFiClientSecure wifiClient;
PubSubClient mqttClient(MQTT_URL, MQTT_PORT, wifiClient);

bool publish_config(const String& topic, const byte* bytes, size_t len) {
    Serial.print("Publishing message to '");
    Serial.print(topic);
    Serial.println("'");

    if (! mqttClient.beginPublish(topic.c_str(), len, false)) {
        Serial.println("Error beginning publish");
        return false;
    }
    if (len != mqttClient.write(bytes, len)) {
        Serial.println("Mismatch in written message length!");
        return false;
    }
    if (! mqttClient.endPublish()) {
        Serial.println("Error ending publish");
        return false;
    }
    return true;
}

bool publish_message(const String& topic, const String& msg) {
    Serial.print("Publishing message to '");
    Serial.print(topic);
    Serial.println("'");

    bool res = mqttClient.publish(topic.c_str(), msg.c_str());
    for (uint8_t retries = 0; retries < MAX_RETRY_COUNT && ! res; retries++) {
        Serial.println("Error publishing message");
        Serial.println(" try again in 5 seconds");
        delay(RETRY_DELAY);
        res = mqttClient.publish(topic.c_str(), msg.c_str());
    }
    return res;
}

bool announce_device() {
    Serial.println("Announcing devices");
    StaticJsonDocument<400> configuration;
    byte configBytes[400];
    size_t msgLen;
    uint8_t retries;

    // Flow sensor
    configuration["name"] = "BWT - Water";
    configuration["unit_of_measurement"] = "L";
    configuration["unique_id"] = "bwtwatersensor1";
    configuration["expire_after"] = 90000; // 25 hours
    configuration["availability_topic"] = HASS_TOPIC_BASE + "W/available";
    configuration["state_topic"] = HASS_TOPIC_BASE + "/state";
    configuration["value_template"] = "{{ value_json.water}}";

    msgLen = measureJson(configuration);
    serializeJson(configuration, configBytes, msgLen + 1);

    bool res = publish_config(HASS_TOPIC_BASE + "W/config", configBytes, msgLen);
    for (uint8_t retries = 0; retries < MAX_RETRY_COUNT && ! res; retries++) {
        Serial.println(" try again in 5 seconds");
        delay(RETRY_DELAY);
        res = publish_config(HASS_TOPIC_BASE + "W/config", configBytes, msgLen);
    }
    if (!res) {
        return false;
    }
    serializeJsonPretty(configuration, Serial);
    Serial.println();
    if (! publish_message(HASS_TOPIC_BASE + "W/available", "online")) {
        return false;
    }

    // Battery sensor
    configuration["device_class"] = "battery";
    configuration["name"] = "BWT - Battery Level";
    configuration["unit_of_measurement"] = "%";
    configuration["unique_id"] = "bwtbatsensor1";
    configuration["expire_after"] = 172800; // 2 days
    configuration["availability_topic"] = HASS_TOPIC_BASE + "B/available";
    configuration["state_topic"] = HASS_TOPIC_BASE + "/state";
    configuration["value_template"] = "{{ value_json.battery }}";

    msgLen = measureJson(configuration);
    serializeJson(configuration, configBytes, msgLen + 1);
    res = publish_config(HASS_TOPIC_BASE + "B/config", configBytes, msgLen);
    for (uint8_t retries = 0; retries < MAX_RETRY_COUNT && ! res; retries++) {
        Serial.println(" try again in 5 seconds");
        delay(RETRY_DELAY);
        res = publish_config(HASS_TOPIC_BASE + "B/config", configBytes, msgLen);
    }
    if (!res) {
        return false;
    }
    serializeJsonPretty(configuration, Serial);
    Serial.println();
    if (! publish_message(HASS_TOPIC_BASE + "B/available", "online")) {
        return false;
    }
    return true;
}

void sleep() {
    Serial.println("Entering deep sleep\n\n");
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(WAKEUP_PERIOD));
    esp_deep_sleep_start();
}

void setup() {
    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    delay(100);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_TIMER) {
        printf("Not timed wakeup, initializing ULP\n");
        init_ulp_program();
    } else {
        printf("ULP wakeup, saving pulse count\n");
        update_pulse_count();
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(WIFI_SSID);
        WiFi.begin(WIFI_SSID, WIFI_PSK);

        // attempt to connect to Wifi network:
        bool res = WiFi.status() == WL_CONNECTED;
        for (uint8_t retries = 0; retries < MAX_RETRY_COUNT && ! res; retries++) {
            Serial.print(".");
            delay(RETRY_DELAY);
            res = WiFi.status() == WL_CONNECTED;
        }
        if (!res) {
            Serial.println("WiFi connection failed, sleeping");
            sleep();
        }

        Serial.print("Connected to ");
        Serial.println(WIFI_SSID);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        wifiClient.setCACert(Lets_Encrypt_R3);

        res = mqttClient.connected();
        for (uint8_t retries = 0; retries < MAX_RETRY_COUNT && ! res; retries++) {
            Serial.print("Attempting MQTT connection...");
            if (mqttClient.connect("iot_bwt", MQTT_USER, MQTT_PASSWORD)) {
                Serial.println("connected");
            } else {
                Serial.print("failed, rc=");
                Serial.print(mqttClient.state());
                Serial.println(" try again in 5 seconds");
                // Wait 5 seconds before retrying
                delay(RETRY_DELAY);
            }
            res = mqttClient.connected();
        }
        if (!res) {
            Serial.println("MQTT connection failed, sleeping");
            sleep();
        }

        if (!announce_device()) {
            Serial.println("Announcing devices failed, sleeping");
            sleep();
        }

        StaticJsonDocument<46> state;
        String stateStr;

        state["battery"] = (uint8_t)((float)(analogRead(A0) - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100);
        state["water"] = 0;
        stateStr = "";
        serializeJson(state, stateStr);
        res = publish_message(HASS_TOPIC_BASE + "/state", stateStr);
        serializeJsonPretty(state, Serial);
        Serial.println();

        if (res) {
            // Publish actual usage second
            const char* namespc = "pulsecnt";
            const char* count_key = "count";

            ESP_ERROR_CHECK( nvs_flash_init() );
            nvs_handle handle;
            ESP_ERROR_CHECK( nvs_open(namespc, NVS_READWRITE, &handle));
            uint32_t pulse_count = 0;
            esp_err_t err = nvs_get_u32(handle, count_key, &pulse_count);
            assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);

            state["water"] = (float)pulse_count / 400;
            stateStr = "";
            serializeJson(state, stateStr);
            if (publish_message(HASS_TOPIC_BASE + "/state", stateStr)) {
                ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, 0));
                ESP_ERROR_CHECK(nvs_commit(handle));
            }
            serializeJsonPretty(state, Serial);
            Serial.println();

            nvs_close(handle);
        }
    }

    sleep();
}

static void init_ulp_program(void) {
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */
    gpio_num_t gpio_num = PIN_FLOW_SENSOR;
    int rtcio_num = PIN_RTC;
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");

    /*
     * Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter = 3;
    ulp_debounce_max_count = 3;
    ulp_next_edge = 0;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
    // ulp_edge_count_to_wake_up = 200;

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    // rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_hold_en(gpio_num);

    /*
     * Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    /*
     * Set ULP wake up period
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1)
     * Maximum flow rate is about 6 L/min which corresponds to 50Hz
     * To be on the safe side consider double that
     * That means a wake up period that is able to detect 10ms pulses
     */
    ulp_set_wakeup_period(0, 10.0 / (ulp_debounce_counter + 1) * 1000);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static void update_pulse_count(void) {
    const char* namespc = "pulsecnt";
    const char* count_key = "count";

    ESP_ERROR_CHECK(nvs_flash_init());
    nvs_handle handle;
    ESP_ERROR_CHECK(nvs_open(namespc, NVS_READWRITE, &handle));
    uint32_t pulse_count = 0;
    esp_err_t err = nvs_get_u32(handle, count_key, &pulse_count);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    printf("Read pulse count from NVS: %5d\n", pulse_count);

    /* ULP program counts signal edges, convert that to the number of pulses */
    uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
    /* In case of an odd number of edges, keep one until next time */
    ulp_edge_count = ulp_edge_count % 2;
    printf("Pulse count from ULP: %5d\n", pulse_count_from_ulp);

    /* Save the new pulse count to NVS */
    pulse_count += pulse_count_from_ulp;
    ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, pulse_count));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    printf("Wrote updated pulse count to NVS: %5d\n", pulse_count);
}

void loop() { }
