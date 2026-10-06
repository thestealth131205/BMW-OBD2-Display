#include "SD_MMC.h"

#define EXAMPLE_MAX_CHAR_SIZE    64
#define MOUNT_POINT "/sdcard"

static const char *SD_TAG = "SD";

uint32_t Flash_Size = 0;
uint32_t SDCard_Size = 0;
// Wird am Ende von SD_Init() in jedem Fall gesetzt (Erfolg oder Fehlschlag) -
// andere Tasks, die vor dem Lesen/Schreiben auf der Karte sicher wissen
// muessen, dass der Mount-Versuch durchgelaufen ist (z.B. sntp_sync.c),
// koennen darauf pollen statt auf eine feste Verzoegerung zu vertrauen.
volatile bool SD_Init_Done = false;
// Zeiger auf die zuletzt erfolgreich gemountete Karte - wird fuer SD_Format()
// gebraucht, da esp_vfs_fat_sdcard_format() diesen Handle erwartet (nicht nur
// den Mount-Pfad).
static sdmmc_card_t *s_card = NULL;

esp_err_t SD_Card_D3_EN(void)
{
    Set_EXIO(TCA9554_EXIO4,true);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}
esp_err_t SD_Card_D3_Dis(void)
{
    Set_EXIO(TCA9554_EXIO4,false);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(SD_TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(SD_TAG, "File written");

    return ESP_OK;
}

esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(SD_TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(SD_TAG, "Read from file: '%s'", line);

    return ESP_OK;
}


void SD_Init(void)
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // format_if_mount_failed bewusst aus: Ein voller Formatierungsversuch einer
    // mehrere-GB-Karte blockiert app_main() teils ueber eine Minute - laeuft das
    // waehrend dieser Zeit erneut in den (noch ungeklaerten) BLE-Reset, bleibt die
    // Karte dauerhaft halb formatiert und der naechste Boot versucht sofort wieder
    // zu formatieren -> Dauerbootloop, noch vor SD_Log_Init()/dem Boot-Logo. Bei
    // fehlgeschlagenem Mount jetzt stattdessen SD-Karte ueberspringen (SDCard_Size
    // bleibt 0, SD_Log_Init() tut dann nichts) und mit dem Rest des Bootvorgangs
    // fortfahren - Karte muss am PC manuell neu formatiert werden.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(SD_TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing production applications.
    ESP_LOGI(SD_TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    SD_Card_D3_EN();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;          // 1-wire  / 4-wire   slot_config.width = 4;

    slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
    slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
    slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
    slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
    slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
    slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
    
    // Enable internal pullups on enabled pins. The internal pullups are insufficient however, please make sure 10k external pullups are connected on the bus. This is for debug / example purpose only.
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;



    ESP_LOGI(SD_TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(SD_TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        SD_Init_Done = true;
        return;
    }
    ESP_LOGI(SD_TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    SDCard_Size = ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024);
    s_card = card;
    SD_Init_Done = true;
}
bool SD_EnsureMounted(void)
{
    if (SDCard_Size == 0) {
        SD_Init();
    }
    return SDCard_Size > 0;
}

esp_err_t SD_Format(void)
{
    const char mount_point[] = MOUNT_POINT;

    if (SDCard_Size > 0 && s_card != NULL) {
        // Bereits gemountet: esp_vfs_fat_sdcard_format() haengt selbst aus,
        // formatiert und mountet danach wieder ein.
        esp_err_t err = esp_vfs_fat_sdcard_format(mount_point, s_card);
        if (err == ESP_OK) {
            SDCard_Size = ((uint64_t) s_card->csd.capacity) * s_card->csd.sector_size / (1024 * 1024);
        }
        return err;
    }

    // Keine Karte gemountet (z.B. unformatiert/beschaedigt, Mount beim Boot
    // bewusst ohne format_if_mount_failed gelaufen, siehe SD_Init()) - hier
    // einmalig erzwungen formatieren statt wie beim normalen Boot aufzugeben.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    SD_Card_D3_EN();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
    slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
    slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
    slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
    slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
    slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    sdmmc_card_t *card = NULL;
    esp_err_t err = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (err == ESP_OK) {
        s_card = card;
        SDCard_Size = ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024);
    }
    return err;
}

void Flash_Searching(void)
{
    if(esp_flash_get_physical_size(NULL, &Flash_Size) == ESP_OK)
    {
        Flash_Size = Flash_Size / (uint32_t)(1024 * 1024);
        printf("Flash size: %ld MB\n", Flash_Size);
    }
    else{
        printf("Get flash size failed\n");
    }
}