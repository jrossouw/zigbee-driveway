/*
 *  This sketch implements my driveway controller
 *
 */

#include <Arduino.h>
#include <Elog.h>
#include <zbhci.h>
#include <hci_display.h>
#include <OneButton.h>
#include <Wire.h>
#include "esp_task_wdt.h"
#include "DHT.h"

#define BASIC_CLUSTER_ID                0x0000
#define ONOFF_CLUSTER_ID                0x0006
#define MEASURE_LUMINANCE_CLUSTER_ID    0x0400
#define MEASURE_TEMPERATURE_CLUSTER_ID  0x0402
#define MEASURE_HUMIDITY_CLUSTER_ID     0x0405
#define OCCUPANCY_SENSING_CLUSTER_ID    0x0406
#define IAS_ZONE_CLUSTER_ID             0x0500
#define MEASURE_ELECTRICAL_CLUSTER_ID   0x0B04

#define CONFIG_ZIGBEE_MODULE_PIN 0
#define CONFIG_USR_BUTTON_PIN 2
#define CONFIG_BLUE_LED_PIN 3
#define CONFIG_DHT_PIN 4

#define REPORTING_PERIOD 10

#define DHT_TYPE DHT22

Elog elog;
DHT dht(CONFIG_DHT_PIN, DHT_TYPE);

const uint8_t au8ManufacturerName[] = { 13,'L','I','L','Y','G','O', '_', 'D', 'R', 'I', 'V', 'E', 'W' };

QueueHandle_t msg_queue;

/**
 * Initialize a new OneButton instance for a button
 * connected to digital pin 4 and GND, which is active low
 * and uses the internal pull-up resistor.
 */
OneButton btn = OneButton(CONFIG_USR_BUTTON_PIN,     // Input pin for the button
    true,  // Button is active LOW
    true); // Enable internal pull-up resistor


uint8_t ledState = 0;
uint8_t netState = 0;
uint8_t autoReport = 0;

void handleClick(void) {
    ts_DstAddr sDstAddr;

    sDstAddr.u16DstAddr = 0x0000;
    if (netState == 1) {
        int16_t h = 4500;
        int16_t t = 2370;
        int16_t v = 1250;
        int8_t m = random(100) > 50 ? 0 : 1;
        ledState = !ledState;
        digitalWrite(CONFIG_BLUE_LED_PIN, ledState);
        elog.log(INFO, "Sensor readings: temp=%f, humi=%f", (float)(t / 100.0), (float)(h / 100.0));
        elog.log(INFO, "Sending report");
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, ONOFF_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_BOOLEAN, 1, &ledState);
        delay(100);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, MEASURE_TEMPERATURE_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_DATA16, 2, (uint8_t*)&t);
        delay(100);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, MEASURE_HUMIDITY_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_DATA16, 2, (uint8_t*)&h);
        delay(100);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, MEASURE_ELECTRICAL_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_DATA16, 2, (uint8_t*)&v);
        delay(100);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, IAS_ZONE_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_BOOLEAN, 1, (uint8_t*)&m);
        delay(100);
    }
    else {
        elog.log(WARNING, "Not joined the zigbee network");
    }

    digitalWrite(CONFIG_BLUE_LED_PIN, true);
    delay(2000);
    digitalWrite(CONFIG_BLUE_LED_PIN, false);
}

void reportTask(void* pvParameters) {
    ts_DstAddr sDstAddr;
    int16_t h, t, v;
    int8_t m;

    sDstAddr.u16DstAddr = 0x0000;
    while (autoReport) {
        digitalWrite(CONFIG_BLUE_LED_PIN, true);

        h = 4000 + random(2000);
        t = 1000 + random(3000);
        v = 4000 + random(2000); 
        m = random(100) > 50 ? 0 : 1;
        elog.log(INFO, "Report task: temp=%f, humi=%f, v=%3.1f", (float)(t / 100.0f), (float)(h / 100.0f), (float)(v / 100.0f));
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, ONOFF_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_BOOLEAN, 1, &ledState);
        delay(100);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, MEASURE_TEMPERATURE_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_DATA16, 2, (uint8_t*)&t);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, MEASURE_HUMIDITY_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_DATA16, 2, (uint8_t*)&h);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, MEASURE_ELECTRICAL_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_DATA16, 2, (uint8_t*)&v);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, IAS_ZONE_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_DATA8, 1, (uint8_t*)&m);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        digitalWrite(CONFIG_BLUE_LED_PIN, false);
        vTaskDelay(REPORTING_PERIOD * 1000 / portTICK_PERIOD_MS);
    }
}

void handleDoubleClick(void) {
    if (autoReport == 0) {
        autoReport = 1;
        xTaskCreatePinnedToCore(
            reportTask,
            "report",      // A name just for humans
            4096,          // This stack size can be checked & adjusted by reading the Stack Highwater
            NULL,
            6,             // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
            NULL,
            ARDUINO_RUNNING_CORE);
    }
    else {
        autoReport = 0;
        elog.log(INFO, "Stop report task");
        delay(1000);
    }
}

void handleLongPress() {
    if (netState == 0) {
        elog.log(INFO, "Joining the zigbee network");
        zbhci_BdbCommissionSteer();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else if (netState == 1) {
        elog.log(INFO, "Leaving the zigbee network");
        zbhci_BdbFactoryReset();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        netState = 0;
    }
}


void zbhciTask(void* pvParameters) {
    ts_HciMsg sHciMsg;
    ts_DstAddr sDstAddr;

    while (1) {
        bzero(&sHciMsg, sizeof(sHciMsg));
        if (xQueueReceive(msg_queue, &sHciMsg, portMAX_DELAY)) {
            switch (sHciMsg.u16MsgType) {
            case ZBHCI_CMD_ACKNOWLEDGE:
                displayAcknowledg(&sHciMsg.uPayload.sAckPayload);
                break;

            case ZBHCI_CMD_NETWORK_STATE_RSP:
                if (sHciMsg.uPayload.sNetworkStateRspPayloasd.u16NwkAddr == 0x0000) {
                    zbhci_BdbFactoryReset();
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    zbhci_NetworkStateReq();
                }
                else if (sHciMsg.uPayload.sNetworkStateRspPayloasd.u16NwkAddr != 0xFFFF) {
                    netState = 1;
                }
                break;

            case ZBHCI_CMD_NETWORK_STATE_REPORT:
                netState = 1;
                sDstAddr.u16DstAddr = 0x0000;
                zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, BASIC_CLUSTER_ID, 0x0005, ZCL_DATA_TYPE_CHAR_STR, sizeof(au8ManufacturerName), (uint8_t*)&au8ManufacturerName);
                break;

            case ZBHCI_CMD_ZCL_ONOFF_CMD_RCV:
                if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 0) {
                    digitalWrite(CONFIG_BLUE_LED_PIN, LOW);
                    ledState = 0;
                }
                else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 1) {
                    digitalWrite(CONFIG_BLUE_LED_PIN, HIGH);
                    ledState = 1;
                }
                else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 2) {
                    ledState = !ledState;
                    digitalWrite(CONFIG_BLUE_LED_PIN, ledState);
                }
                zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, ONOFF_CLUSTER_ID, 0x0000, ZCL_DATA_TYPE_BOOLEAN, 1, &ledState);

                break;

            default:
                elog.log(WARNING, "Unhandled u16MsgType %d\n", sHciMsg.u16MsgType);
                break;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {

    Serial.begin(115200);
    delay(10);

    elog.addSerialLogging(Serial, "APP", INFO); // Enable serial logging. We want only INFO or lower logleve.
    elog.log(ALERT, "Sketch started");

    pinMode(CONFIG_ZIGBEE_MODULE_PIN, OUTPUT);
    digitalWrite(CONFIG_ZIGBEE_MODULE_PIN, HIGH);
    delay(500);

   
    pinMode(CONFIG_BLUE_LED_PIN, OUTPUT);
    digitalWrite(CONFIG_BLUE_LED_PIN, LOW);


    dht.begin();

    btn.attachClick(handleClick);
    btn.attachDoubleClick(handleDoubleClick);
    btn.setPressTicks(3000);
    btn.attachLongPressStart(handleLongPress);

    msg_queue = xQueueCreate(10, sizeof(ts_HciMsg));
    zbhci_Init(msg_queue);

    xTaskCreatePinnedToCore(
        zbhciTask,
        "zbhci",       // A name just for humans
        4096,          // This stack size can be checked & adjusted by reading the Stack Highwater
        NULL,
        5,             // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        NULL,
        ARDUINO_RUNNING_CORE);

    // zbhci_BdbFactoryReset();
    delay(100);
    zbhci_NetworkStateReq();

}


void loop() {
    btn.tick();
    if (netState == 1 && autoReport == 0) {
        handleDoubleClick();
    }
}
