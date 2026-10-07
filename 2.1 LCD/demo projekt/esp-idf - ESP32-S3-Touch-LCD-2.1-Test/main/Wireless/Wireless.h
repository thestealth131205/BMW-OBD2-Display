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

// Vollstaendiges An-/Abschalten des Bluetooth-Controllers + Bluedroid-Stacks
// (nicht nur Scan/Verbindung von ble_obd.c). esp_bluedroid_init()/
// esp_bt_controller_init() reservieren dauerhaft mehrere 10 KB internes DRAM
// (ACL-/GATT-/HCI-Puffer), die alleine durch Scan-Stop oder GATT-Disconnect
// NICHT freigegeben werden - genau das hat esp_wifi_init() beim WLAN-Update/
// Hotspot-Zeitabgleich trotz vermeintlich ausreichend freiem Heap mit
// ESP_ERR_NO_MEM scheitern lassen. BLE_OBD_Suspend()/_Resume() rufen diese
// Funktionen auf, um den Controller fuer die kurze WiFi-Nutzung komplett
// abzubauen und danach wieder aufzubauen.
bool Wireless_BT_Deinit(void);
bool Wireless_BT_Reinit(void);