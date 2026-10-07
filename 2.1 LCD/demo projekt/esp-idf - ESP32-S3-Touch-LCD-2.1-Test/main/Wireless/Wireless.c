#include "Wireless.h"
#include "esp_heap_caps.h"
#include "sd_log.h"

uint16_t BLE_NUM = 0;
uint16_t WIFI_NUM = 0;
bool Scan_finish = 0;

bool WiFi_Scan_Finish = 0;
bool BLE_Scan_Finish = 0;
volatile bool BLE_Stack_Ready = 0;

// esp_wifi_init() reserviert RX/TX-Puffer im internen DRAM (bei den Default-
// Groessen mehrere 10 KB), die auch nach esp_wifi_stop() bestehen bleiben -
// der Treiber ist dann "gestoppt", aber weiterhin initialisiert. Dieses Flag
// haelt den tatsaechlichen Init-Zustand fest, damit WIFI_Init() (Boot-Scan),
// ota_web.c (AP) und sntp_sync.c (STA) sich den Treiber teilen koennen, ohne
// ihn versehentlich doppelt zu initialisieren oder zu frueh zu deinitialisieren.
static bool s_wifi_driver_up = false;

bool Wireless_WiFi_Init_If_Needed(void)
{
    if (s_wifi_driver_up) return true;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        // Vorher nur per printf (UART) sichtbar - landete nie im SD-Log, das
        // einzige, was man im Auto ohne angeschlossenen Rechner einsehen
        // kann. "fehlgeschlagen" ohne Fehlercode war daher nicht diagnostizierbar.
        SD_Log("WIFI: esp_wifi_init (Re-Init) -> %s (intern frei=%u)",
               esp_err_to_name(err), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        return false;
    }
    // Siehe Kommentar in WIFI_Init(): keine NVS-Nutzung fuer WiFi-Config
    // noetig, vermeidet unnoetige Flash-Schreibzugriffe/-Fehler.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    s_wifi_driver_up = true;
    return true;
}

void Wireless_WiFi_Deinit(void)
{
    if (!s_wifi_driver_up) return;
    esp_err_t stop_err = esp_wifi_stop();
    esp_err_t err = esp_wifi_deinit();
    SD_Log("WIFI: esp_wifi_stop -> %s, esp_wifi_deinit -> %s (intern frei=%u)",
           esp_err_to_name(stop_err), esp_err_to_name(err),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    // Schlug esp_wifi_deinit() fehl, ist der Treiber laut ESP-IDF weiterhin
    // initialisiert - das Flag muss das widerspiegeln, sonst wuerde
    // Wireless_WiFi_Init_If_Needed() spaeter faelschlich ein zweites
    // esp_wifi_init() auf einen bereits initialisierten Treiber versuchen
    // (das schlaegt dann mit ESP_ERR_WIFI_NOT_STOPPED/ESP_FAIL fehl - exakt
    // das beobachtete "Wireless_WiFi_Init_If_Needed() fehlgeschlagen").
    if (err == ESP_OK) {
        s_wifi_driver_up = false;
    }
}

void Wireless_Init(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    // WiFi
    xTaskCreatePinnedToCore(
        WIFI_Init, 
        "WIFI task",
        4096, 
        NULL, 
        3, 
        NULL, 
        0);
    // BLE
    xTaskCreatePinnedToCore(
        BLE_Init, 
        "BLE task",
        4096, 
        NULL, 
        2, 
        NULL, 
        0);
}

void WIFI_Init(void *arg)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    Wireless_WiFi_Init_If_Needed();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    WIFI_NUM = WIFI_Scan();
    printf("WIFI:%d\r\n",WIFI_NUM);

    // Nach dem einmaligen Boot-Scan WiFi komplett abschalten UND deinitiali-
    // sieren, statt den Treiber nur zu stoppen. esp_wifi_init() allokiert
    // RX/TX-Puffer im internen DRAM (mehrere 10 KB), die mit blossem
    // esp_wifi_stop() bestehen blieben - die ganze Laufzeit ueber, parallel
    // zur dauerhaft aktiven Bluedroid-BLE-OBD-Verbindung, obwohl WiFi nur fuer
    // den kurzen WLAN-Update- bzw. Hotspot-Zeitabgleich-Vorgang gebraucht
    // wird. ota_web.c/sntp_sync.c rufen jetzt selbst Wireless_WiFi_Init_If_
    // Needed() auf, wenn sie WiFi tatsaechlich brauchen, und holen sich den
    // Treiber dann frisch zurueck.
    Wireless_WiFi_Deinit();

    vTaskDelete(NULL);
}
uint16_t WIFI_Scan(void)
{
    uint16_t ap_count = 0;
    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    WiFi_Scan_Finish =1;
    if(BLE_Scan_Finish == 1)
        Scan_finish = 1;
    return ap_count;
}


#define GATTC_TAG "GATTC_TAG"
#define SCAN_DURATION 5  
#define MAX_DISCOVERED_DEVICES 100 

typedef struct {
    uint8_t address[6];
    bool is_valid;
} discovered_device_t;

static discovered_device_t discovered_devices[MAX_DISCOVERED_DEVICES];
static size_t num_discovered_devices = 0;
static size_t num_devices_with_name = 0;


static bool is_device_discovered(const uint8_t *addr) {
    for (size_t i = 0; i < num_discovered_devices; i++) {
        if (memcmp(discovered_devices[i].address, addr, 6) == 0) {
            return true;
        }
    }
    return false;
}


static void add_device_to_list(const uint8_t *addr) {
    if (num_discovered_devices < MAX_DISCOVERED_DEVICES) {
        memcpy(discovered_devices[num_discovered_devices].address, addr, 6);
        discovered_devices[num_discovered_devices].is_valid = true;
        num_discovered_devices++;
    }
}

static bool extract_device_name(const uint8_t *adv_data, uint8_t adv_data_len, char *device_name, size_t max_name_len) {
    size_t offset = 0;
    while (offset < adv_data_len) {
        if (adv_data[offset] == 0) break;

        uint8_t length = adv_data[offset];
        if (length == 0 || offset + length > adv_data_len) break; 

        uint8_t type = adv_data[offset + 1];
        if (type == ESP_BLE_AD_TYPE_NAME_CMPL || type == ESP_BLE_AD_TYPE_NAME_SHORT) {
            if (length > 1 && length - 1 < max_name_len) {
                memcpy(device_name, &adv_data[offset + 2], length - 1);
                device_name[length - 1] = '\0'; 
                return true;
            } else {
                return false;
            }
        }
        offset += length + 1;
    }
    return false;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    static char device_name[100];

    // Alle GAP-Events zusaetzlich an den BLE-OBD-Client weiterreichen
    // (Bluedroid kennt nur einen globalen GAP-Callback).
    BLE_OBD_gap_event(event, param);

    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (!BLE_Scan_Finish && param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                if (!is_device_discovered(param->scan_rst.bda)) {
                    add_device_to_list(param->scan_rst.bda);
                    BLE_NUM++; 

                    if (extract_device_name(param->scan_rst.ble_adv, param->scan_rst.adv_data_len, device_name, sizeof(device_name))) {
                        num_devices_with_name++;
                        // printf("Found device: %02X:%02X:%02X:%02X:%02X:%02X\n        Name: %s\n        RSSI: %d\r\n",
                        //          param->scan_rst.bda[0], param->scan_rst.bda[1],
                        //          param->scan_rst.bda[2], param->scan_rst.bda[3],
                        //          param->scan_rst.bda[4], param->scan_rst.bda[5],
                        //          device_name, param->scan_rst.rssi);
                        // printf("\r\n");
                    } else {
                        // printf("Found device: %02X:%02X:%02X:%02X:%02X:%02X\n        Name: Unknown\n        RSSI: %d\r\n",
                        //          param->scan_rst.bda[0], param->scan_rst.bda[1],
                        //          param->scan_rst.bda[2], param->scan_rst.bda[3],
                        //          param->scan_rst.bda[4], param->scan_rst.bda[5],
                        //          param->scan_rst.rssi);
                        // printf("\r\n");
                    }
                }
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(GATTC_TAG, "Scan stopped. Total devices found: %d (with names: %d)", BLE_NUM, num_devices_with_name);
            break;
        default:
            break;
    }
}

void BLE_Init(void *arg)
{
    // Kurze Verzoegerung, damit die restliche Peripherie-Initialisierung in
    // app_main (Display/QSPI, I2C, SD) nicht zeitgleich mit dem schweren
    // BT-Controller-/Bluedroid-Start um Bus-/CPU-Zeit konkurriert.
    vTaskDelay(pdMS_TO_TICKS(700));

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);                                            
    if (ret) {
        printf("%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));        
        return;}
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);                                           
    if (ret) {
        printf("%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));            
        return;}
    ret = esp_bluedroid_init();                                                                 
    if (ret) {
        printf("%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));               
        return;}
    ret = esp_bluedroid_enable();                                                               
    if (ret) {
        printf("%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));             
        return;}

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);                                            
    if (ret){
        printf("%s gap register error, error code = %x\n", __func__, ret);                      
        return;
    }
    // Stack ist bereit: der BLE-OBD-Client kann sofort mit Suche/Verbindung
    // starten (parallel zur Start-Animation). Den Geraetezaehler der
    // Demo-Seite fuettert dessen Dauerscan waehrend der ersten Sekunden.
    BLE_Stack_Ready = 1;
    BLE_Scan();
    vTaskDelete(NULL);

}
uint16_t BLE_Scan(void)
{
    // Kein eigener Scan mehr: der BLE-OBD-Client scannt dauerhaft. Wir zaehlen
    // nur SCAN_DURATION Sekunden lang die dabei gefundenen Geraete mit.
    vTaskDelay(SCAN_DURATION * 1000 / portTICK_PERIOD_MS);
    BLE_Scan_Finish = 1;
    if(WiFi_Scan_Finish == 1)
        Scan_finish = 1;
    return BLE_NUM;
}
