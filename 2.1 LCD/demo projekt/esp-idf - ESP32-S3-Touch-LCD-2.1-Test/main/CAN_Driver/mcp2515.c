#include "mcp2515.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MCP2515";

// SPI3_HOST verwenden: SPI2_HOST belegt der ST7701S-LCD-Init.
#define MCP2515_SPI_HOST   SPI3_HOST
#define MCP2515_SPI_CLK_HZ (8 * 1000 * 1000)   // 8 MHz SPI, MCP2515 kann bis 10 MHz

// --- SPI-Kommandos ---
#define CMD_RESET        0xC0
#define CMD_READ         0x03
#define CMD_WRITE        0x02
#define CMD_RTS          0x80   // Request-to-send (| TXBn-Maske)
#define CMD_READ_STATUS  0xA0
#define CMD_BIT_MODIFY   0x05

// --- Register ---
#define REG_CANCTRL   0x0F
#define REG_CANSTAT   0x0E
#define REG_CNF3      0x28
#define REG_CNF2      0x29
#define REG_CNF1      0x2A
#define REG_CANINTE   0x2B
#define REG_CANINTF   0x2C
#define REG_RXB0CTRL  0x60
#define REG_RXB0SIDH  0x61
#define REG_RXB1CTRL  0x70
#define REG_RXB1SIDH  0x71
#define REG_RXM0SIDH  0x20
#define REG_RXM1SIDH  0x24
#define REG_TXB0CTRL  0x30
#define REG_TXB0SIDH  0x31

// CANINTF-Bits
#define INTF_RX0IF  0x01
#define INTF_RX1IF  0x02

// CANCTRL-Modi (obere 3 Bits)
#define MODE_NORMAL  0x00
#define MODE_CONFIG  0x80
#define MODE_MASK    0xE0

static spi_device_handle_t s_spi;

// --- CNF-Werte fuer 500 kbit/s (autowp/arduino-mcp2515) ---
#if MCP2515_XTAL_MHZ == 16
#define CNF1_500K  0x00
#define CNF2_500K  0xF0
#define CNF3_500K  0x86
#else  // 8 MHz
#define CNF1_500K  0x00
#define CNF2_500K  0x90
#define CNF3_500K  0x82
#endif

static void reg_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[3] = { CMD_WRITE, reg, val };
    spi_transaction_t t = { .length = 3 * 8, .tx_buffer = tx };
    spi_device_transmit(s_spi, &t);
}

static uint8_t reg_read(uint8_t reg)
{
    uint8_t tx[3] = { CMD_READ, reg, 0x00 };
    uint8_t rx[3] = { 0 };
    spi_transaction_t t = { .length = 3 * 8, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_transmit(s_spi, &t);
    return rx[2];
}

static void reg_bit_modify(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t tx[4] = { CMD_BIT_MODIFY, reg, mask, val };
    spi_transaction_t t = { .length = 4 * 8, .tx_buffer = tx };
    spi_device_transmit(s_spi, &t);
}

static void mcp_reset(void)
{
    uint8_t tx = CMD_RESET;
    spi_transaction_t t = { .length = 8, .tx_buffer = &tx };
    spi_device_transmit(s_spi, &t);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static esp_err_t set_mode(uint8_t mode)
{
    reg_bit_modify(REG_CANCTRL, MODE_MASK, mode);
    for (int i = 0; i < 10; i++) {
        if ((reg_read(REG_CANSTAT) & MODE_MASK) == mode) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_FAIL;
}

esp_err_t mcp2515_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = MCP2515_PIN_MOSI,
        .miso_io_num = MCP2515_PIN_MISO,
        .sclk_io_num = MCP2515_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(MCP2515_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = MCP2515_SPI_CLK_HZ,
        .mode = 0,                       // MCP2515: SPI-Mode 0,0
        .spics_io_num = MCP2515_PIN_CS,
        .queue_size = 4,
    };
    err = spi_bus_add_device(MCP2515_SPI_HOST, &devcfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    mcp_reset();

    if (set_mode(MODE_CONFIG) != ESP_OK) {
        ESP_LOGE(TAG, "MCP2515 antwortet nicht (Verdrahtung/Quarz pruefen)");
        return ESP_FAIL;
    }

    // 500 kbit/s
    reg_write(REG_CNF1, CNF1_500K);
    reg_write(REG_CNF2, CNF2_500K);
    reg_write(REG_CNF3, CNF3_500K);

    // Nur RX-Interrupts aktivieren (INT-Pin), Rest aus
    reg_write(REG_CANINTE, INTF_RX0IF | INTF_RX1IF);
    reg_write(REG_CANINTF, 0x00);

    // Accept-All: Masken auf 0 -> keine Bits werden verglichen, alle IDs durch.
    // RXB0: Rollover auf RXB1 erlauben, kein Filter (RXM=11).
    reg_write(REG_RXB0CTRL, 0x64);   // RXM=11 (alle), BUKT=1 (Rollover)
    reg_write(REG_RXB1CTRL, 0x60);   // RXM=11 (alle)
    for (int i = 0; i < 4; i++) {
        reg_write(REG_RXM0SIDH + i, 0x00);
        reg_write(REG_RXM1SIDH + i, 0x00);
    }

    if (set_mode(MODE_NORMAL) != ESP_OK) {
        ESP_LOGE(TAG, "Wechsel in NORMAL-Mode fehlgeschlagen");
        return ESP_FAIL;
    }

    if (MCP2515_PIN_INT >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << MCP2515_PIN_INT,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&io);
    }

    ESP_LOGI(TAG, "MCP2515 bereit (500 kbit/s, %d MHz Quarz)", MCP2515_XTAL_MHZ);
    return ESP_OK;
}

// Liest einen RX-Puffer (base = REG_RXB0SIDH oder REG_RXB1SIDH) via READ.
static bool read_rx_buffer(uint8_t sidh_reg, mcp2515_frame_t *out)
{
    uint8_t tx[2 + 13] = { CMD_READ, sidh_reg };
    uint8_t rx[2 + 13] = { 0 };
    spi_transaction_t t = { .length = sizeof(tx) * 8, .tx_buffer = tx, .rx_buffer = rx };
    if (spi_device_transmit(s_spi, &t) != ESP_OK) return false;

    uint8_t *b = &rx[2];   // b[0]=SIDH b[1]=SIDL b[2]=EID8 b[3]=EID0 b[4]=DLC b[5..]=data
    out->id = ((uint32_t)b[0] << 3) | (b[1] >> 5);   // Standard-11-Bit-ID
    out->dlc = b[4] & 0x0F;
    if (out->dlc > 8) out->dlc = 8;
    memcpy(out->data, &b[5], out->dlc);
    return true;
}

bool mcp2515_receive(mcp2515_frame_t *out)
{
    uint8_t intf = reg_read(REG_CANINTF);
    if (intf & INTF_RX0IF) {
        bool ok = read_rx_buffer(REG_RXB0SIDH, out);
        reg_bit_modify(REG_CANINTF, INTF_RX0IF, 0x00);
        return ok;
    }
    if (intf & INTF_RX1IF) {
        bool ok = read_rx_buffer(REG_RXB1SIDH, out);
        reg_bit_modify(REG_CANINTF, INTF_RX1IF, 0x00);
        return ok;
    }
    return false;
}

esp_err_t mcp2515_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8) dlc = 8;
    // TXB0 laden: SIDH/SIDL/EID8/EID0/DLC + Daten
    uint8_t buf[2 + 5 + 8] = { CMD_WRITE, REG_TXB0SIDH };
    buf[2] = (uint8_t)(id >> 3);            // SIDH
    buf[3] = (uint8_t)((id & 0x07) << 5);   // SIDL (Standard-Frame)
    buf[4] = 0x00;                          // EID8
    buf[5] = 0x00;                          // EID0
    buf[6] = dlc;                           // DLC (kein RTR)
    memcpy(&buf[7], data, dlc);
    spi_transaction_t t = { .length = (7 + dlc) * 8, .tx_buffer = buf };
    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err != ESP_OK) return err;

    // Senden anstossen (RTS TXB0)
    uint8_t rts = CMD_RTS | 0x01;
    spi_transaction_t t2 = { .length = 8, .tx_buffer = &rts };
    return spi_device_transmit(s_spi, &t2);
}
