#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#define LCD_HOST_SPI SPI2_HOST

#define TFT_SCLK 6
#define TFT_MOSI 7
#define TFT_DC   2
#define TFT_CS   10
#define TFT_RST  -1
#define TFT_BL   3

#define LCD_WIDTH  240
#define LCD_HEIGHT 240

#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0
#define CYAN    0x07FF
#define MAGENTA 0xF81F

static const char *TAG = "lcd_test";
static spi_device_handle_t lcd_spi;

static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level(TFT_DC, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };

    ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
}

static void lcd_data(const void *data, int len)
{
    if (len <= 0) return;

    gpio_set_level(TFT_DC, 1);

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data
    };

    ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
}

static void lcd_byte(uint8_t data)
{
    lcd_data(&data, 1);
}

static void lcd_reset(void)
{
#if TFT_RST >= 0
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
#else
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(150));
#endif
}

static void lcd_set_window(int x0, int y0, int x1, int y1)
{
    uint8_t data[4];

    lcd_cmd(0x2A);
    data[0] = x0 >> 8;
    data[1] = x0 & 0xFF;
    data[2] = x1 >> 8;
    data[3] = x1 & 0xFF;
    lcd_data(data, 4);

    lcd_cmd(0x2B);
    data[0] = y0 >> 8;
    data[1] = y0 & 0xFF;
    data[2] = y1 >> 8;
    data[3] = y1 & 0xFF;
    lcd_data(data, 4);

    lcd_cmd(0x2C);
}

static void lcd_init_gc9a01(void)
{
    lcd_reset();

    lcd_cmd(0xEF);
    lcd_cmd(0xEB); lcd_byte(0x14);
    lcd_cmd(0xFE);
    lcd_cmd(0xEF);

    lcd_cmd(0xEB); lcd_byte(0x14);
    lcd_cmd(0x84); lcd_byte(0x40);
    lcd_cmd(0x85); lcd_byte(0xFF);
    lcd_cmd(0x86); lcd_byte(0xFF);
    lcd_cmd(0x87); lcd_byte(0xFF);
    lcd_cmd(0x88); lcd_byte(0x0A);
    lcd_cmd(0x89); lcd_byte(0x21);
    lcd_cmd(0x8A); lcd_byte(0x00);
    lcd_cmd(0x8B); lcd_byte(0x80);
    lcd_cmd(0x8C); lcd_byte(0x01);
    lcd_cmd(0x8D); lcd_byte(0x01);
    lcd_cmd(0x8E); lcd_byte(0xFF);
    lcd_cmd(0x8F); lcd_byte(0xFF);

    lcd_cmd(0xB6);
    lcd_byte(0x00);
    lcd_byte(0x20);

    lcd_cmd(0x36);
    lcd_byte(0x00);

    lcd_cmd(0x3A);
    lcd_byte(0x05);

    lcd_cmd(0x90);
    lcd_byte(0x08);
    lcd_byte(0x08);
    lcd_byte(0x08);
    lcd_byte(0x08);

    lcd_cmd(0xBD); lcd_byte(0x06);
    lcd_cmd(0xBC); lcd_byte(0x00);

    lcd_cmd(0xFF);
    lcd_byte(0x60);
    lcd_byte(0x01);
    lcd_byte(0x04);

    lcd_cmd(0xC3); lcd_byte(0x13);
    lcd_cmd(0xC4); lcd_byte(0x13);
    lcd_cmd(0xC9); lcd_byte(0x22);

    lcd_cmd(0xBE); lcd_byte(0x11);

    lcd_cmd(0xE1);
    lcd_byte(0x10);
    lcd_byte(0x0E);

    lcd_cmd(0xDF);
    lcd_byte(0x21);
    lcd_byte(0x0C);
    lcd_byte(0x02);

    lcd_cmd(0xF0);
    lcd_byte(0x45);
    lcd_byte(0x09);
    lcd_byte(0x08);
    lcd_byte(0x08);
    lcd_byte(0x26);
    lcd_byte(0x2A);

    lcd_cmd(0xF1);
    lcd_byte(0x43);
    lcd_byte(0x70);
    lcd_byte(0x72);
    lcd_byte(0x36);
    lcd_byte(0x37);
    lcd_byte(0x6F);

    lcd_cmd(0xF2);
    lcd_byte(0x45);
    lcd_byte(0x09);
    lcd_byte(0x08);
    lcd_byte(0x08);
    lcd_byte(0x26);
    lcd_byte(0x2A);

    lcd_cmd(0xF3);
    lcd_byte(0x43);
    lcd_byte(0x70);
    lcd_byte(0x72);
    lcd_byte(0x36);
    lcd_byte(0x37);
    lcd_byte(0x6F);

    lcd_cmd(0xED);
    lcd_byte(0x1B);
    lcd_byte(0x0B);

    lcd_cmd(0xAE); lcd_byte(0x77);
    lcd_cmd(0xCD); lcd_byte(0x63);

    lcd_cmd(0x70);
    lcd_byte(0x07);
    lcd_byte(0x07);
    lcd_byte(0x04);
    lcd_byte(0x0E);
    lcd_byte(0x0F);
    lcd_byte(0x09);
    lcd_byte(0x07);
    lcd_byte(0x08);
    lcd_byte(0x03);

    lcd_cmd(0xE8); lcd_byte(0x34);

    lcd_cmd(0x62);
    lcd_byte(0x18);
    lcd_byte(0x0D);
    lcd_byte(0x71);
    lcd_byte(0xED);
    lcd_byte(0x70);
    lcd_byte(0x70);
    lcd_byte(0x18);
    lcd_byte(0x0F);
    lcd_byte(0x71);
    lcd_byte(0xEF);
    lcd_byte(0x70);
    lcd_byte(0x70);

    lcd_cmd(0x63);
    lcd_byte(0x18);
    lcd_byte(0x11);
    lcd_byte(0x71);
    lcd_byte(0xF1);
    lcd_byte(0x70);
    lcd_byte(0x70);
    lcd_byte(0x18);
    lcd_byte(0x13);
    lcd_byte(0x71);
    lcd_byte(0xF3);
    lcd_byte(0x70);
    lcd_byte(0x70);

    lcd_cmd(0x64);
    lcd_byte(0x28);
    lcd_byte(0x29);
    lcd_byte(0xF1);
    lcd_byte(0x01);
    lcd_byte(0xF1);
    lcd_byte(0x00);
    lcd_byte(0x07);

    lcd_cmd(0x66);
    lcd_byte(0x3C);
    lcd_byte(0x00);
    lcd_byte(0xCD);
    lcd_byte(0x67);
    lcd_byte(0x45);
    lcd_byte(0x45);
    lcd_byte(0x10);
    lcd_byte(0x00);
    lcd_byte(0x00);
    lcd_byte(0x00);

    lcd_cmd(0x67);
    lcd_byte(0x00);
    lcd_byte(0x3C);
    lcd_byte(0x00);
    lcd_byte(0x00);
    lcd_byte(0x00);
    lcd_byte(0x01);
    lcd_byte(0x54);
    lcd_byte(0x10);
    lcd_byte(0x32);
    lcd_byte(0x98);

    lcd_cmd(0x74);
    lcd_byte(0x10);
    lcd_byte(0x85);
    lcd_byte(0x80);
    lcd_byte(0x00);
    lcd_byte(0x00);
    lcd_byte(0x4E);
    lcd_byte(0x00);

    lcd_cmd(0x98);
    lcd_byte(0x3E);
    lcd_byte(0x07);

    lcd_cmd(0x21);
    lcd_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void lcd_fill(uint16_t colour)
{
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    static uint8_t line[LCD_WIDTH * 2];

    uint8_t hi = colour >> 8;
    uint8_t lo = colour & 0xFF;

    for (int i = 0; i < LCD_WIDTH; i++) {
        line[i * 2] = hi;
        line[i * 2 + 1] = lo;
    }

    gpio_set_level(TFT_DC, 1);

    for (int y = 0; y < LCD_HEIGHT; y++) {
        spi_transaction_t t = {
            .length = LCD_WIDTH * 16,
            .tx_buffer = line
        };

        ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-IDF LCD colour test starting");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TFT_DC) | (1ULL << TFT_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(TFT_BL, 1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST_SPI, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = TFT_CS,
        .queue_size = 1
    };

    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST_SPI, &devcfg, &lcd_spi));

    lcd_init_gc9a01();

    while (1) {
        ESP_LOGI(TAG, "RED");
        lcd_fill(RED);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "GREEN");
        lcd_fill(GREEN);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "BLUE");
        lcd_fill(BLUE);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "WHITE");
        lcd_fill(WHITE);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "YELLOW");
        lcd_fill(YELLOW);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "CYAN");
        lcd_fill(CYAN);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "MAGENTA");
        lcd_fill(MAGENTA);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "BLACK");
        lcd_fill(BLACK);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}