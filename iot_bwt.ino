#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#include <driver/rtc_io.h>
#include <esp32/ulp.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <ulptool.h>

#include "config.h"
#include "ulp_main.h"

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

    if (! mqttClient.beginPublish(topic.c_str(), len, true)) {
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

    bool res = mqttClient.publish(topic.c_str(), msg.c_str(), true);
    for (uint8_t retries = 0; retries < MAX_RETRY_COUNT && ! res; retries++) {
        Serial.println("Error publishing message");
        Serial.println(" try again in 5 seconds");
        delay(RETRY_DELAY);
        res = mqttClient.publish(topic.c_str(), msg.c_str(), true);
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

    // Battery sensor
    configuration["device_class"] = "battery";
    configuration["name"] = "BWT - Battery Level";
    configuration["unit_of_measurement"] = "%";
    configuration["unique_id"] = "bwtbatsensor1";
    configuration["expire_after"] = 172800; // 2 days
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

        /*
         * Workaround Home Assistant' utility meter behaviour
         * It expects a monotonically increasing value and cumulates the
         * difference for each submission. To trick this into adding all the
         * submitted values straight up, we can send a 0 value before sending
         * the real one, this way the difference is always compared to 0, so
         * the correct value gets added.
         */
        state["battery"] = (uint8_t)((float)(analogRead(A0) - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100);
        if (state["battery"] > 100) state["battery"] = 100;
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
