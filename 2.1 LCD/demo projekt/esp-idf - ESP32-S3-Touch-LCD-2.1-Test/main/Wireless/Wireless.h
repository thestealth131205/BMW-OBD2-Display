#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "nvs_flash.h" 
#include "esp_log.h"

#include <stdio.h>
#include <string.h>  // For memcpy
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "ble_obd.h"



extern uint16_t BLE_NUM;
extern uint16_t WIFI_NUM;
extern bool Scan_finish;
extern bool WiFi_Scan_Finish;
extern volatile bool BLE_Stack_Ready;

void Wireless_Init(void);
void WIFI_Init(void *arg);
uint16_t WIFI_Scan(void);
void BLE_Init(void *arg);
uint16_t BLE_Scan(void);

// Zentrales An-/Abschalten des WiFi-Treibers selbst (nicht nur Start/Stop):
// esp_wifi_init() allokiert RX/TX-Puffer im internen DRAM, die auch nach
// esp_wifi_stop() bestehen bleiben - WiFi wird aber nur fuer den kurzen
// WLAN-Update- bzw. Hotspot-Zeitabgleich-Vorgang gebraucht, den Rest der
// Laufzeit laeuft nur BLE-OBD. ota_web.c/sntp_sync.c rufen diese beiden statt
// esp_wifi_init()/esp_wifi_deinit() direkt auf, damit der Zustand an einer
// Stelle verwaltet wird.
bool Wireless_WiFi_Init_If_Needed(void);
void Wireless_WiFi_Deinit(void);