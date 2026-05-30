
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// =========================================================
// ESP32-C3 AIRCRAFT RADAR + TOUCH SELECT
// Board: ESP32-C3 1.28" 240x240 round GC9A01 LCD + CST816 touch
// LCD pins and touch pins confirmed.
// =========================================================

static const char *TAG = "aircraft_touch_radar";

// =========================================================
// USER SETTINGS
// =========================================================

// Defaults used only if no saved settings exist.
// Normal values are loaded from NVS flash or set through the setup portal.
#define DEFAULT_g_wifi_ssid      "TP-Link_515C"
#define DEFAULT_g_wifi_password  "PUT_YOUR_g_wifi_password_HERE"
#define DEFAULT_POSTCODE       "S81 7NH"
#define DEFAULT_RANGE_MILES    20.0

#define CONFIG_AP_SSID         "RADAR-SETUP"
#define CONFIG_AP_PASSWORD     "12345678"

#define HTTP_RESPONSE_BUFFER_SIZE 24576
#define AIRCRAFT_RESPONSE_BUFFER_SIZE 8192
#define MAX_AIRCRAFT 20
#define TOUCH_SELECT_RADIUS_PX 26
#define AIRCRAFT_UPDATE_INTERVAL_MS 15000

static char g_wifi_ssid[33] = DEFAULT_g_wifi_ssid;
static char g_wifi_password[65] = DEFAULT_g_wifi_password;
static char g_postcode[16] = DEFAULT_POSTCODE;
static char g_postcode_encoded[48] = "S81%207NH";
static double g_radar_range_miles = DEFAULT_RANGE_MILES;
static double g_radar_range_km = DEFAULT_RANGE_MILES * 1.609344;
static bool g_has_saved_config = false;

// =========================================================
// LCD PINS - CONFIRMED WORKING
// =========================================================

#define LCD_HOST_SPI SPI2_HOST

#define TFT_SCLK 6
#define TFT_MOSI 7
#define TFT_DC   2
#define TFT_CS   10
#define TFT_RST  -1
#define TFT_BL   3

#define LCD_WIDTH  240
#define LCD_HEIGHT 240

#define RADAR_CX 120
#define RADAR_CY 120
#define RADAR_RADIUS 104
#define RADAR_SWEEP_INTERVAL_MS 35
#define RADAR_SWEEP_STEP_DEG 4.0
#define RANGE_TOUCH_MINUS_X_MIN 0
#define RANGE_TOUCH_MINUS_X_MAX 42
#define RANGE_TOUCH_PLUS_X_MIN 198
#define RANGE_TOUCH_PLUS_X_MAX 239
#define RANGE_TOUCH_Y_MIN 82
#define RANGE_TOUCH_Y_MAX 158

// =========================================================
// TOUCH PINS - CONFIRMED WORKING
// =========================================================

#define TOUCH_SDA      4
#define TOUCH_SCL      5
#define TOUCH_RST      1
#define TOUCH_INT      0
#define TOUCH_ADDR_PRIMARY 0x15

#define TOUCH_REG_GESTURE  0x01
#define TOUCH_REG_FINGER   0x02
#define TOUCH_REG_XH       0x03

// =========================================================
// COLOURS RGB565
// =========================================================

#define BLACK    0x0000
#define WHITE    0xFFFF
#define RED      0xF800
#define GREEN    0x07E0
#define BLUE     0x001F
#define YELLOW   0xFFE0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define DARK     0x1082
#define DIMGREEN 0x0320
#define AMBER    0xFD20
#define POLICE_BLUE 0x04FF

static spi_device_handle_t lcd_spi;
static uint8_t active_touch_addr = 0;
static uint8_t g_radar_frame[LCD_WIDTH * LCD_HEIGHT * 2];

// =========================================================
// DATA STRUCTURES
// =========================================================

typedef struct {
    char callsign[16];
    char aircraft_class[12];
    double lat;
    double lon;
    double altitude_m;
    double velocity_ms;
    double heading_deg;
    double distance_km;
    double bearing_deg;
    int screen_x;
    int screen_y;
    bool valid;
} aircraft_t;

typedef struct {
    aircraft_t aircraft[MAX_AIRCRAFT];
    int count;
} aircraft_list_t;

static SemaphoreHandle_t g_aircraft_mutex = NULL;
static aircraft_list_t g_aircraft_latest = {0};
static bool g_aircraft_latest_ok = false;
static bool g_aircraft_has_update = false;
static uint32_t g_aircraft_update_seq = 0;
static double g_home_lat = 0.0;
static double g_home_lon = 0.0;

// =========================================================
// WIFI STATE
// =========================================================

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_WIFI_RETRY     10

static EventGroupHandle_t wifi_event_group;
static int wifi_retry_count = 0;

// =========================================================
// SIMPLE 5x7 FONT
// =========================================================

static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43},
};

// =========================================================
// LCD FUNCTIONS
// =========================================================

static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level(TFT_DC, 0);
    spi_transaction_t t = {.length = 8, .tx_buffer = &cmd};
    ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
}

static void lcd_data(const void *data, int len)
{
    if (len <= 0) return;
    gpio_set_level(TFT_DC, 1);
    spi_transaction_t t = {.length = len * 8, .tx_buffer = data};
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
    data[0] = x0 >> 8; data[1] = x0 & 0xFF;
    data[2] = x1 >> 8; data[3] = x1 & 0xFF;
    lcd_data(data, 4);

    lcd_cmd(0x2B);
    data[0] = y0 >> 8; data[1] = y0 & 0xFF;
    data[2] = y1 >> 8; data[3] = y1 & 0xFF;
    lcd_data(data, 4);

    lcd_cmd(0x2C);
}

static void lcd_draw_pixel(int x, int y, uint16_t colour)
{
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;
    lcd_set_window(x, y, x, y);
    uint8_t data[2] = {colour >> 8, colour & 0xFF};
    lcd_data(data, 2);
}

static void lcd_fill(uint16_t colour)
{
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    static uint8_t line[LCD_WIDTH * 2];

    for (int i = 0; i < LCD_WIDTH; i++) {
        line[i * 2] = colour >> 8;
        line[i * 2 + 1] = colour & 0xFF;
    }

    gpio_set_level(TFT_DC, 1);

    for (int y = 0; y < LCD_HEIGHT; y++) {
        spi_transaction_t t = {.length = LCD_WIDTH * 16, .tx_buffer = line};
        ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
    }
}

static void lcd_draw_char(int x, int y, char c, uint16_t colour, int scale)
{
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c < 32 || c > 90) c = ' ';

    const uint8_t *bitmap = font5x7[c - 32];

    for (int col = 0; col < 5; col++) {
        uint8_t line = bitmap[col];
        for (int row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                for (int sx = 0; sx < scale; sx++) {
                    for (int sy = 0; sy < scale; sy++) {
                        lcd_draw_pixel(x + col * scale + sx, y + row * scale + sy, colour);
                    }
                }
            }
        }
    }
}

static void lcd_draw_text(int x, int y, const char *text, uint16_t colour, int scale)
{
    int cursor_x = x;
    while (*text) {
        lcd_draw_char(cursor_x, y, *text, colour, scale);
        cursor_x += 6 * scale;
        text++;
    }
}

static int lcd_text_width(const char *text, int scale)
{
    return (int)strlen(text) * 6 * scale;
}

static void format_int_with_commas(int value, char *out, int out_len)
{
    char raw[16];
    char temp[24];
    int raw_len = 0;
    int temp_len = 0;
    bool negative = value < 0;

    if (negative) {
        value = -value;
    }

    snprintf(raw, sizeof(raw), "%d", value);
    raw_len = (int)strlen(raw);

    if (negative && temp_len < (int)sizeof(temp) - 1) {
        temp[temp_len++] = '-';
    }

    for (int i = 0; i < raw_len && temp_len < (int)sizeof(temp) - 1; i++) {
        temp[temp_len++] = raw[i];

        int remaining = raw_len - i - 1;
        if (remaining > 0 && remaining % 3 == 0 && temp_len < (int)sizeof(temp) - 1) {
            temp[temp_len++] = ',';
        }
    }

    temp[temp_len] = '\0';
    snprintf(out, out_len, "%s", temp);
}

static void lcd_draw_text_centered(int y, const char *text, uint16_t colour, int scale)
{
    int w = lcd_text_width(text, scale);
    int x = (LCD_WIDTH - w) / 2;
    if (x < 0) x = 0;
    lcd_draw_text(x, y, text, colour, scale);
}

static void lcd_draw_circle(int xc, int yc, int r, uint16_t colour)
{
    int x = 0;
    int y = r;
    int d = 3 - 2 * r;

    while (y >= x) {
        lcd_draw_pixel(xc + x, yc + y, colour);
        lcd_draw_pixel(xc - x, yc + y, colour);
        lcd_draw_pixel(xc + x, yc - y, colour);
        lcd_draw_pixel(xc - x, yc - y, colour);
        lcd_draw_pixel(xc + y, yc + x, colour);
        lcd_draw_pixel(xc - y, yc + x, colour);
        lcd_draw_pixel(xc + y, yc - x, colour);
        lcd_draw_pixel(xc - y, yc - x, colour);

        x++;

        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else {
            d = d + 4 * x + 6;
        }
    }
}

static void lcd_blit_radar_frame(void)
{
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    gpio_set_level(TFT_DC, 1);

    const int rows_per_chunk = 40;

    for (int y = 0; y < LCD_HEIGHT; y += rows_per_chunk) {
        int rows = LCD_HEIGHT - y;
        if (rows > rows_per_chunk) {
            rows = rows_per_chunk;
        }

        spi_transaction_t t = {
            .length = LCD_WIDTH * rows * 16,
            .tx_buffer = &g_radar_frame[y * LCD_WIDTH * 2]
        };
        ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
    }
}

static void radar_fb_set_pixel(int x, int y, uint16_t colour)
{
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;

    int offset = ((y * LCD_WIDTH) + x) * 2;
    g_radar_frame[offset] = colour >> 8;
    g_radar_frame[offset + 1] = colour & 0xFF;
}

static void radar_fb_clear(uint16_t colour)
{
    uint8_t high = colour >> 8;
    uint8_t low = colour & 0xFF;

    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        g_radar_frame[i * 2] = high;
        g_radar_frame[i * 2 + 1] = low;
    }
}

static void radar_fb_draw_char(int x, int y, char c, uint16_t colour, int scale)
{
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c < 32 || c > 90) c = ' ';

    const uint8_t *bitmap = font5x7[c - 32];

    for (int col = 0; col < 5; col++) {
        uint8_t line = bitmap[col];
        for (int row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                for (int sx = 0; sx < scale; sx++) {
                    for (int sy = 0; sy < scale; sy++) {
                        radar_fb_set_pixel(x + col * scale + sx, y + row * scale + sy, colour);
                    }
                }
            }
        }
    }
}

static void radar_fb_draw_text(int x, int y, const char *text, uint16_t colour, int scale)
{
    int cursor_x = x;
    while (*text) {
        radar_fb_draw_char(cursor_x, y, *text, colour, scale);
        cursor_x += 6 * scale;
        text++;
    }
}

static void radar_fb_draw_text_centered(int y, const char *text, uint16_t colour, int scale)
{
    int w = lcd_text_width(text, scale);
    int x = (LCD_WIDTH - w) / 2;
    if (x < 0) x = 0;
    radar_fb_draw_text(x, y, text, colour, scale);
}

static void radar_fb_draw_line(int x0, int y0, int x1, int y1, uint16_t colour)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        radar_fb_set_pixel(x0, y0, colour);
        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void radar_fb_draw_circle(int xc, int yc, int r, uint16_t colour)
{
    int x = 0;
    int y = r;
    int d = 3 - 2 * r;

    while (y >= x) {
        radar_fb_set_pixel(xc + x, yc + y, colour);
        radar_fb_set_pixel(xc - x, yc + y, colour);
        radar_fb_set_pixel(xc + x, yc - y, colour);
        radar_fb_set_pixel(xc - x, yc - y, colour);
        radar_fb_set_pixel(xc + y, yc + x, colour);
        radar_fb_set_pixel(xc - y, yc + x, colour);
        radar_fb_set_pixel(xc + y, yc - x, colour);
        radar_fb_set_pixel(xc - y, yc - x, colour);

        x++;

        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else {
            d = d + 4 * x + 6;
        }
    }
}

static void radar_fb_fill_circle(int xc, int yc, int r, uint16_t colour)
{
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            if ((x * x) + (y * y) <= r * r) {
                radar_fb_set_pixel(xc + x, yc + y, colour);
            }
        }
    }
}

static void radar_fb_draw_sweep(double angle_deg)
{
    static const uint16_t trail_colours[] = {
        GREEN,
        0x05E0,
        0x04A0,
        DIMGREEN,
        0x0220,
        0x0140
    };

    const int trail_count = (int)(sizeof(trail_colours) / sizeof(trail_colours[0]));
    const double band_width_deg = 7.0;
    const double fill_step_deg = 1.5;

    for (int i = trail_count - 1; i >= 0; i--) {
        double newest_angle = angle_deg - (double)i * band_width_deg;
        double oldest_angle = newest_angle - band_width_deg;

        for (double trail_angle = oldest_angle; trail_angle <= newest_angle; trail_angle += fill_step_deg) {
            double wrapped_angle = trail_angle;

            while (wrapped_angle < 0.0) wrapped_angle += 360.0;
            while (wrapped_angle >= 360.0) wrapped_angle -= 360.0;

            double angle_rad = wrapped_angle * M_PI / 180.0;
            int x = RADAR_CX + (int)round(sin(angle_rad) * RADAR_RADIUS);
            int y = RADAR_CY - (int)round(cos(angle_rad) * RADAR_RADIUS);

            radar_fb_draw_line(RADAR_CX, RADAR_CY, x, y, trail_colours[i]);
        }
    }
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
    lcd_byte(0x45); lcd_byte(0x09); lcd_byte(0x08);
    lcd_byte(0x08); lcd_byte(0x26); lcd_byte(0x2A);

    lcd_cmd(0xF1);
    lcd_byte(0x43); lcd_byte(0x70); lcd_byte(0x72);
    lcd_byte(0x36); lcd_byte(0x37); lcd_byte(0x6F);

    lcd_cmd(0xF2);
    lcd_byte(0x45); lcd_byte(0x09); lcd_byte(0x08);
    lcd_byte(0x08); lcd_byte(0x26); lcd_byte(0x2A);

    lcd_cmd(0xF3);
    lcd_byte(0x43); lcd_byte(0x70); lcd_byte(0x72);
    lcd_byte(0x36); lcd_byte(0x37); lcd_byte(0x6F);

    lcd_cmd(0xED);
    lcd_byte(0x1B);
    lcd_byte(0x0B);

    lcd_cmd(0xAE); lcd_byte(0x77);
    lcd_cmd(0xCD); lcd_byte(0x63);

    lcd_cmd(0x70);
    lcd_byte(0x07); lcd_byte(0x07); lcd_byte(0x04);
    lcd_byte(0x0E); lcd_byte(0x0F); lcd_byte(0x09);
    lcd_byte(0x07); lcd_byte(0x08); lcd_byte(0x03);

    lcd_cmd(0xE8); lcd_byte(0x34);

    lcd_cmd(0x62);
    lcd_byte(0x18); lcd_byte(0x0D); lcd_byte(0x71); lcd_byte(0xED);
    lcd_byte(0x70); lcd_byte(0x70); lcd_byte(0x18); lcd_byte(0x0F);
    lcd_byte(0x71); lcd_byte(0xEF); lcd_byte(0x70); lcd_byte(0x70);

    lcd_cmd(0x63);
    lcd_byte(0x18); lcd_byte(0x11); lcd_byte(0x71); lcd_byte(0xF1);
    lcd_byte(0x70); lcd_byte(0x70); lcd_byte(0x18); lcd_byte(0x13);
    lcd_byte(0x71); lcd_byte(0xF3); lcd_byte(0x70); lcd_byte(0x70);

    lcd_cmd(0x64);
    lcd_byte(0x28); lcd_byte(0x29); lcd_byte(0xF1); lcd_byte(0x01);
    lcd_byte(0xF1); lcd_byte(0x00); lcd_byte(0x07);

    lcd_cmd(0x66);
    lcd_byte(0x3C); lcd_byte(0x00); lcd_byte(0xCD); lcd_byte(0x67);
    lcd_byte(0x45); lcd_byte(0x45); lcd_byte(0x10); lcd_byte(0x00);
    lcd_byte(0x00); lcd_byte(0x00);

    lcd_cmd(0x67);
    lcd_byte(0x00); lcd_byte(0x3C); lcd_byte(0x00); lcd_byte(0x00);
    lcd_byte(0x00); lcd_byte(0x01); lcd_byte(0x54); lcd_byte(0x10);
    lcd_byte(0x32); lcd_byte(0x98);

    lcd_cmd(0x74);
    lcd_byte(0x10); lcd_byte(0x85); lcd_byte(0x80); lcd_byte(0x00);
    lcd_byte(0x00); lcd_byte(0x4E); lcd_byte(0x00);

    lcd_cmd(0x98);
    lcd_byte(0x3E);
    lcd_byte(0x07);

    lcd_cmd(0x21);
    lcd_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void lcd_init_hardware(void)
{
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
}

// =========================================================
// SOFTWARE I2C TOUCH
// =========================================================

static void i2c_delay(void)
{
    esp_rom_delay_us(4);
}

static void sda_high(void)
{
    gpio_set_direction(TOUCH_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(TOUCH_SDA, 1);
}

static void sda_low(void)
{
    gpio_set_direction(TOUCH_SDA, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(TOUCH_SDA, 0);
}

static void scl_high(void)
{
    gpio_set_direction(TOUCH_SCL, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(TOUCH_SCL, 1);
}

static void scl_low(void)
{
    gpio_set_direction(TOUCH_SCL, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(TOUCH_SCL, 0);
}

static int sda_read(void)
{
    gpio_set_direction(TOUCH_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    return gpio_get_level(TOUCH_SDA);
}

static void soft_i2c_init_pins(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << TOUCH_SDA) | (1ULL << TOUCH_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&conf));

    sda_high();
    scl_high();
    i2c_delay();
}

static void soft_i2c_start(void)
{
    sda_high();
    scl_high();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
    i2c_delay();
}

static void soft_i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_high();
    i2c_delay();
    sda_high();
    i2c_delay();
}

static bool soft_i2c_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) sda_high();
        else sda_low();

        i2c_delay();
        scl_high();
        i2c_delay();
        scl_low();
        i2c_delay();

        byte <<= 1;
    }

    sda_high();
    i2c_delay();
    scl_high();
    i2c_delay();

    bool ack = (sda_read() == 0);

    scl_low();
    i2c_delay();

    return ack;
}

static uint8_t soft_i2c_read_byte(bool ack)
{
    uint8_t byte = 0;
    sda_high();

    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        scl_high();
        i2c_delay();

        if (sda_read()) byte |= 1;

        scl_low();
        i2c_delay();
    }

    if (ack) sda_low();
    else sda_high();

    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
    i2c_delay();
    sda_high();

    return byte;
}

static bool soft_i2c_probe(uint8_t addr)
{
    soft_i2c_start();
    bool ack = soft_i2c_write_byte((addr << 1) | 0);
    soft_i2c_stop();
    return ack;
}

static bool soft_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    soft_i2c_start();

    if (!soft_i2c_write_byte((addr << 1) | 0)) {
        soft_i2c_stop();
        return false;
    }

    if (!soft_i2c_write_byte(reg)) {
        soft_i2c_stop();
        return false;
    }

    soft_i2c_start();

    if (!soft_i2c_write_byte((addr << 1) | 1)) {
        soft_i2c_stop();
        return false;
    }

    for (int i = 0; i < len; i++) {
        buf[i] = soft_i2c_read_byte(i < len - 1);
    }

    soft_i2c_stop();
    return true;
}

static void touch_reset_controller(void)
{
#if TOUCH_RST >= 0
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << TOUCH_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&rst_conf));

    gpio_set_level(TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
#endif
}

static bool touch_init(void)
{
#if TOUCH_INT >= 0
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << TOUCH_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&int_conf);
#endif

    touch_reset_controller();
    soft_i2c_init_pins();

    uint8_t candidates[] = {0x15, 0x38, 0x14, 0x5D, 0x2E};

    for (int i = 0; i < (int)sizeof(candidates); i++) {
        uint8_t addr = candidates[i];
        ESP_LOGI(TAG, "Probing touch address 0x%02X", addr);

        if (soft_i2c_probe(addr)) {
            active_touch_addr = addr;
            ESP_LOGI(TAG, "Touch ACK at 0x%02X", active_touch_addr);
            return true;
        }
    }

    ESP_LOGE(TAG, "No touch controller found on software I2C");
    return false;
}

static bool touch_read_raw(int *x, int *y, int *gesture, int *fingers)
{
    if (active_touch_addr == 0) return false;

    uint8_t data[6] = {0};

    if (!soft_i2c_read_regs(active_touch_addr, TOUCH_REG_GESTURE, data, sizeof(data))) {
        return false;
    }

    int gesture_raw = data[0];
    int fingers_raw = data[1] & 0x0F;
    int x_raw = ((data[2] & 0x0F) << 8) | data[3];
    int y_raw = ((data[4] & 0x0F) << 8) | data[5];

    if (gesture) *gesture = gesture_raw;
    if (fingers) *fingers = fingers_raw;
    if (x) *x = x_raw;
    if (y) *y = y_raw;

    return fingers_raw > 0;
}

static void map_touch_to_screen(int raw_x, int raw_y, int *screen_x, int *screen_y)
{
    int x = raw_x;
    int y = raw_y;

    if (x < 0) x = 0;
    if (x > 239) x = 239;
    if (y < 0) y = 0;
    if (y > 239) y = 239;

    *screen_x = x;
    *screen_y = y;
}


// =========================================================
// UI FORWARD DECLARATIONS
// =========================================================

static void show_status_screen(uint16_t bg, const char *line1, const char *line2);
static void lcd_draw_text_centered(int y, const char *text, uint16_t colour, int scale);

// =========================================================
// SETTINGS / CONFIG PORTAL HELPERS
// =========================================================

static void url_encode_postcode(const char *input, char *output, int output_len)
{
    int j = 0;

    for (int i = 0; input[i] && j < output_len - 1; i++) {
        char c = input[i];

        if (c == ' ') {
            if (j < output_len - 3) {
                output[j++] = '%';
                output[j++] = '2';
                output[j++] = '0';
            }
        } else if ((c >= 'A' && c <= 'Z') ||
                   (c >= 'a' && c <= 'z') ||
                   (c >= '0' && c <= '9')) {
            output[j++] = (char)toupper((unsigned char)c);
        }
    }

    output[j] = '\0';
}

static void settings_apply_derived(void)
{
    url_encode_postcode(g_postcode, g_postcode_encoded, sizeof(g_postcode_encoded));

    if (g_radar_range_miles < 1.0) {
        g_radar_range_miles = 1.0;
    }

    if (g_radar_range_miles > 20.0) {
        g_radar_range_miles = 20.0;
    }

    g_radar_range_km = g_radar_range_miles * 1.609344;
}

static void change_radar_range(int direction)
{
    static const double ranges_miles[] = {5.0, 10.0, 15.0, 20.0};
    const int range_count = sizeof(ranges_miles) / sizeof(ranges_miles[0]);
    int current_index = 0;
    double best_delta = fabs(g_radar_range_miles - ranges_miles[0]);

    for (int i = 1; i < range_count; i++) {
        double delta = fabs(g_radar_range_miles - ranges_miles[i]);
        if (delta < best_delta) {
            best_delta = delta;
            current_index = i;
        }
    }

    int next_index = current_index + (direction >= 0 ? 1 : -1);

    if (next_index >= range_count) {
        next_index = 0;
    } else if (next_index < 0) {
        next_index = range_count - 1;
    }

    double next_range = ranges_miles[next_index];

    g_radar_range_miles = next_range;
    settings_apply_derived();

    ESP_LOGI(TAG, "Radar range changed to %.0f miles", g_radar_range_miles);
}

static bool settings_load(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("radar_cfg", NVS_READONLY, &nvs);

    if (err != ESP_OK) {
        settings_apply_derived();
        return false;
    }

    size_t ssid_len = sizeof(g_wifi_ssid);
    size_t pass_len = sizeof(g_wifi_password);
    size_t post_len = sizeof(g_postcode);

    bool ok = true;

    if (nvs_get_str(nvs, "ssid", g_wifi_ssid, &ssid_len) != ESP_OK) ok = false;
    if (nvs_get_str(nvs, "pass", g_wifi_password, &pass_len) != ESP_OK) ok = false;
    if (nvs_get_str(nvs, "postcode", g_postcode, &post_len) != ESP_OK) ok = false;
    if (nvs_get_blob(nvs, "range", &g_radar_range_miles, &(size_t){sizeof(g_radar_range_miles)}) != ESP_OK) ok = false;

    nvs_close(nvs);

    settings_apply_derived();

    // Password may be blank for open Wi-Fi networks.
    g_has_saved_config = ok && strlen(g_wifi_ssid) > 0 && strlen(g_postcode) > 0;

    ESP_LOGI(TAG, "Config load: %s", g_has_saved_config ? "saved config found" : "using defaults / incomplete");
    ESP_LOGI(TAG, "SSID: %s", g_wifi_ssid);
    ESP_LOGI(TAG, "Postcode: %s encoded %s", g_postcode, g_postcode_encoded);
    ESP_LOGI(TAG, "Range: %.1f miles %.1f km", g_radar_range_miles, g_radar_range_km);

    return g_has_saved_config;
}

static bool settings_save(const char *ssid, const char *pass, const char *postcode, double range_miles)
{
    if (!ssid || !pass || !postcode) {
        return false;
    }

    // Password is allowed to be blank for open Wi-Fi networks.
    // SSID and postcode must still be present.
    if (strlen(ssid) == 0 || strlen(postcode) == 0) {
        return false;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("radar_cfg", NVS_READWRITE, &nvs);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open save failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_str(nvs, "ssid", ssid);
    if (err == ESP_OK) err = nvs_set_str(nvs, "pass", pass);
    if (err == ESP_OK) err = nvs_set_str(nvs, "postcode", postcode);
    if (err == ESP_OK) err = nvs_set_blob(nvs, "range", &range_miles, sizeof(range_miles));
    if (err == ESP_OK) err = nvs_commit(nvs);

    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "settings save failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Settings saved");
    return true;
}

static void url_decode(char *s)
{
    char *src = s;
    char *dst = s;

    while (*src) {
        if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else if (*src == '%' && isxdigit((unsigned char)src[1]) && isxdigit((unsigned char)src[2])) {
            char hex[3] = {src[1], src[2], 0};
            *dst++ = (char)strtol(hex, NULL, 16);
            src += 3;
        } else {
            *dst++ = *src++;
        }
    }

    *dst = '\0';
}

static bool form_get_value(const char *body, const char *key, char *out, int out_len)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "%s=", key);

    const char *p = strstr(body, pattern);

    if (!p) {
        out[0] = '\0';
        return false;
    }

    p += strlen(pattern);

    int i = 0;

    while (*p && *p != '&' && i < out_len - 1) {
        out[i++] = *p++;
    }

    out[i] = '\0';
    url_decode(out);
    return true;
}

static void html_escape(const char *in, char *out, int out_len)
{
    int j = 0;

    for (int i = 0; in[i] && j < out_len - 1; i++) {
        char c = in[i];
        const char *rep = NULL;

        if (c == '&') rep = "&amp;";
        else if (c == '<') rep = "&lt;";
        else if (c == '>') rep = "&gt;";
        else if (c == '"') rep = "&quot;";

        if (rep) {
            int len = strlen(rep);
            if (j + len >= out_len - 1) break;
            strcpy(&out[j], rep);
            j += len;
        } else {
            out[j++] = c;
        }
    }

    out[j] = '\0';
}

static void scan_wifi_options(char *out, int out_len)
{
    out[0] = '\0';

    wifi_scan_config_t scan_config = {0};

    esp_err_t err = esp_wifi_scan_start(&scan_config, true);

    if (err != ESP_OK) {
        snprintf(out, out_len, "<option value=\"%s\">%s</option>", g_wifi_ssid, g_wifi_ssid);
        return;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);

    if (ap_count > 12) {
        ap_count = 12;
    }

    wifi_ap_record_t aps[12] = {0};

    if (ap_count > 0) {
        esp_wifi_scan_get_ap_records(&ap_count, aps);
    }

    int used = 0;

    for (int i = 0; i < ap_count; i++) {
        char ssid_esc[80];
        html_escape((const char *)aps[i].ssid, ssid_esc, sizeof(ssid_esc));

        char option[260];
        snprintf(option,
                 sizeof(option),
                 "<option value=\"%s\" %s>%s (%d dBm)</option>",
                 ssid_esc,
                 strcmp((const char *)aps[i].ssid, g_wifi_ssid) == 0 ? "selected" : "",
                 ssid_esc,
                 aps[i].rssi);

        int len = strlen(option);

        if (used + len < out_len - 1) {
            strcpy(&out[used], option);
            used += len;
        }
    }

    if (used == 0) {
        snprintf(out, out_len, "<option value=\"%s\">%s</option>", g_wifi_ssid, g_wifi_ssid);
    }
}

static esp_err_t config_root_handler(httpd_req_t *req)
{
    char *options = calloc(1, 2400);
    char *page = calloc(1, 5600);

    if (!options || !page) {
        free(options);
        free(page);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    char postcode_esc[64];
    char ssid_esc[80];

    scan_wifi_options(options, 2400);
    html_escape(g_postcode, postcode_esc, sizeof(postcode_esc));
    html_escape(g_wifi_ssid, ssid_esc, sizeof(ssid_esc));

    snprintf(page,
             5600,
             "<!doctype html><html><head>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             "<title>Aircraft Radar Setup</title>"
             "<style>"
             "body{font-family:Arial,sans-serif;background:#101820;color:#e8ffff;margin:0;padding:20px;}"
             ".card{max-width:520px;margin:auto;background:#182c34;border-radius:18px;padding:22px;box-shadow:0 0 20px #0008;}"
             "h1{color:#00ffff;margin-top:0;} label{display:block;margin-top:16px;font-weight:bold;}"
             "input,select{width:100%%;box-sizing:border-box;padding:12px;border-radius:10px;border:0;margin-top:6px;font-size:16px;}"
             "button{width:100%%;padding:14px;border:0;border-radius:12px;background:#00dddd;color:#001014;font-size:18px;font-weight:bold;margin-top:22px;}"
             ".hint{color:#aee;font-size:14px;line-height:1.4;}"
             "</style></head><body><div class='card'>"
             "<h1>Aircraft Radar Setup</h1>"
             "<p class='hint'>Choose your Wi-Fi, enter the password if needed, enter your postcode, then save. "
             "The radar will restart and connect to your network.</p>"
             "<form method='POST' action='/save'>"
             "<label>Wi-Fi Network</label>"
             "<select name='ssid'>%s</select>"
             "<label>Wi-Fi Password</label>"
             "<input name='pass' type='password' placeholder='Leave blank if no password is needed'>"
             "<p class='hint'>Leave this blank for open Wi-Fi networks.</p>"
             "<label>Postcode</label>"
             "<input name='postcode' value='%s' placeholder='S81 7NH'>"
             "<button type='submit'>Save and Restart</button>"
             "</form>"
             "<p class='hint'>Setup hotspot: %s / password %s<br>Device page: http://192.168.4.1</p>"
             "</div></body></html>",
             options,
             postcode_esc,
             CONFIG_AP_SSID,
             CONFIG_AP_PASSWORD);

    httpd_resp_set_type(req, "text/html");
    esp_err_t res = httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);

    free(options);
    free(page);

    return res;
}

static esp_err_t config_save_handler(httpd_req_t *req)
{
    int total_len = req->content_len;

    if (total_len <= 0 || total_len > 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad form length");
        return ESP_FAIL;
    }

    char body[1025] = {0};
    int received = 0;

    while (received < total_len) {
        int ret = httpd_req_recv(req, body + received, total_len - received);

        if (ret <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        received += ret;
    }

    body[received] = '\0';

    char ssid[33] = {0};
    char pass[65] = {0};
    char postcode[16] = {0};

    form_get_value(body, "ssid", ssid, sizeof(ssid));
    form_get_value(body, "pass", pass, sizeof(pass));
    form_get_value(body, "postcode", postcode, sizeof(postcode));

    bool ok = settings_save(ssid, pass, postcode, g_radar_range_miles);

    if (!ok) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Could not save settings");
        return ESP_FAIL;
    }

    const char *page =
        "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>body{font-family:Arial;background:#101820;color:#e8ffff;text-align:center;padding:40px;}"
        ".card{max-width:480px;margin:auto;background:#182c34;border-radius:18px;padding:22px;}</style>"
        "</head><body><div class='card'><h1>Saved</h1><p>Radar is restarting. Reconnect to your normal Wi-Fi.</p></div></body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

static void start_config_portal(void)
{
    show_status_screen(DARK, "CONFIG", "MODE");
    vTaskDelay(pdMS_TO_TICKS(1000));

    lcd_fill(BLACK);
    lcd_draw_text_centered(18, "CONFIG", YELLOW, 2);
    lcd_draw_text_centered(58, "CONNECT TO", WHITE, 1);
    lcd_draw_text_centered(73, "THIS WIFI:", WHITE, 1);
    lcd_draw_text_centered(91, CONFIG_AP_SSID, CYAN, 2);
    lcd_draw_text_centered(124, "USE THIS", WHITE, 1);
    lcd_draw_text_centered(139, "PASSWORD:", WHITE, 1);
    lcd_draw_text_centered(157, CONFIG_AP_PASSWORD, CYAN, 2);
    lcd_draw_text_centered(190, "OPEN IN BROWSER:", WHITE, 1);
    lcd_draw_text_centered(210, "192.168.4.1", CYAN, 2);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {0};

    strncpy((char *)ap_config.ap.ssid, CONFIG_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    strncpy((char *)ap_config.ap.password, CONFIG_AP_PASSWORD, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.ssid_len = strlen(CONFIG_AP_SSID);
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Config portal AP started: %s / %s", CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
    ESP_LOGI(TAG, "Open http://192.168.4.1");

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.lru_purge_enable = true;
    server_config.stack_size = 8192;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &server_config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = config_root_handler,
            .user_ctx = NULL
        };

        httpd_uri_t save_uri = {
            .uri = "/save",
            .method = HTTP_POST,
            .handler = config_save_handler,
            .user_ctx = NULL
        };

        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &save_uri);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// =========================================================
// WIFI
// =========================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi started, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < MAX_WIFI_RETRY) {
            wifi_retry_count++;
            ESP_LOGW(TAG, "Wi-Fi disconnected, retry %d/%d", wifi_retry_count, MAX_WIFI_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "Wi-Fi failed after retries");
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected");
        ESP_LOGI(TAG, "IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_connect(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};

    strncpy((char *)wifi_config.sta.ssid, g_wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, g_wifi_password, sizeof(wifi_config.sta.password) - 1);

    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", g_wifi_ssid);

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(30000)
    );

    return (bits & WIFI_CONNECTED_BIT);
}

// =========================================================
// HTTP
// =========================================================

typedef struct {
    char *buffer;
    int buffer_size;
    int data_len;
    bool truncated;
} http_response_t;

static const char AIRPLANES_LIVE_ROOT_CA[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIICjjCCAjOgAwIBAgIQf/NXaJvCTjAtkOGKQb0OHzAKBggqhkjOPQQDAjBQMSQwIgYDVQQLExtH\n"
    "bG9iYWxTaWduIEVDQyBSb290IENBIC0gUjQxEzARBgNVBAoTCkdsb2JhbFNpZ24xEzARBgNVBAMT\n"
    "Ckdsb2JhbFNpZ24wHhcNMjMxMjEzMDkwMDAwWhcNMjkwMjIwMTQwMDAwWjA7MQswCQYDVQQGEwJV\n"
    "UzEeMBwGA1UEChMVR29vZ2xlIFRydXN0IFNlcnZpY2VzMQwwCgYDVQQDEwNXRTEwWTATBgcqhkjO\n"
    "PQIBBggqhkjOPQMBBwNCAARvzTr+Z1dHTCEDhUDCR127WEcPQMFcF4XGGTfn1XzthkubgdnXGhOl\n"
    "CgP4mMTG6J7/EFmPLCaY9eYmJbsPAvpWo4IBAjCB/zAOBgNVHQ8BAf8EBAMCAYYwHQYDVR0lBBYw\n"
    "FAYIKwYBBQUHAwEGCCsGAQUFBwMCMBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYDVR0OBBYEFJB3kjVn\n"
    "xP+ozKnme9mAeXvMk/k4MB8GA1UdIwQYMBaAFFSwe61FuOJAf/sKbvu+M8k8o4TVMDYGCCsGAQUF\n"
    "BwEBBCowKDAmBggrBgEFBQcwAoYaaHR0cDovL2kucGtpLmdvb2cvZ3NyNC5jcnQwLQYDVR0fBCYw\n"
    "JDAioCCgHoYcaHR0cDovL2MucGtpLmdvb2cvci9nc3I0LmNybDATBgNVHSAEDDAKMAgGBmeBDAEC\n"
    "ATAKBggqhkjOPQQDAgNJADBGAiEAokJL0LgR6SOLR02WWxccAq3ndXp4EMRveXMUVUxMWSMCIQDs\n"
    "pFWa3fj7nLgouSdkcPy1SdOR2AGm9OQWs7veyXsBwA==\n"
    "-----END CERTIFICATE-----\n"
    "-----BEGIN CERTIFICATE-----\n"
    "MIIB3DCCAYOgAwIBAgINAgPlfvU/k/2lCSGypjAKBggqhkjOPQQDAjBQMSQwIgYDVQQLExtHbG9i\n"
    "YWxTaWduIEVDQyBSb290IENBIC0gUjQxEzARBgNVBAoTCkdsb2JhbFNpZ24xEzARBgNVBAMTCkds\n"
    "b2JhbFNpZ24wHhcNMTIxMTEzMDAwMDAwWhcNMzgwMTE5MDMxNDA3WjBQMSQwIgYDVQQLExtHbG9i\n"
    "YWxTaWduIEVDQyBSb290IENBIC0gUjQxEzARBgNVBAoTCkdsb2JhbFNpZ24xEzARBgNVBAMTCkds\n"
    "b2JhbFNpZ24wWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAAS4xnnTj2wlDp8uORkcA6SumuU5BwkW\n"
    "ymOxuYb4ilfBV85C+nOh92VC/x7BALJucw7/xyHlGKSq2XE/qNS5zowdo0IwQDAOBgNVHQ8BAf8E\n"
    "BAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUVLB7rUW44kB/+wpu+74zyTyjhNUwCgYI\n"
    "KoZIzj0EAwIDRwAwRAIgIk90crlgr/HmnKAWBVBfw147bmF0774BxL4YSFlhgjICICadVGNA3jdg\n"
    "UM/I2O2dgq43mLyjj0xMqTQrbO/7lZsm\n"
    "-----END CERTIFICATE-----\n";

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_response_t *response = (http_response_t *)evt->user_data;

    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data_len > 0) {
        if (response->data_len + evt->data_len < response->buffer_size) {
            memcpy(response->buffer + response->data_len, evt->data, evt->data_len);
            response->data_len += evt->data_len;
            response->buffer[response->data_len] = '\0';
        } else {
            int space = response->buffer_size - response->data_len - 1;

            if (space > 0) {
                memcpy(response->buffer + response->data_len, evt->data, space);
                response->data_len += space;
                response->buffer[response->data_len] = '\0';
            }

            response->truncated = true;
        }
    }

    return ESP_OK;
}

static bool http_get(const char *url,
                     char *buffer,
                     int buffer_size,
                     const char *cert_pem,
                     int *status_code,
                     int *bytes_received)
{
    buffer[0] = '\0';

    http_response_t response = {
        .buffer = buffer,
        .buffer_size = buffer_size,
        .data_len = 0,
        .truncated = false
    };

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .user_data = &response,
        .user_agent = "ESP32-Aircraft-Radar/1.0",
        .timeout_ms = cert_pem ? 30000 : 20000,
        .buffer_size = 2048,
        .buffer_size_tx = 1024,
        .cert_pem = cert_pem,
        .cert_len = cert_pem ? strlen(cert_pem) + 1 : 0,
        .common_name = cert_pem ? "api.airplanes.live" : NULL,
        .crt_bundle_attach = cert_pem ? NULL : esp_crt_bundle_attach
    };

    ESP_LOGI(TAG, "HTTP GET: %s", url);

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (!client) {
        ESP_LOGE(TAG, "Failed to create HTTP client");
        return false;
    }

    esp_err_t err = esp_http_client_perform(client);

    *status_code = esp_http_client_get_status_code(client);
    *bytes_received = response.data_len;

    ESP_LOGI(TAG, "HTTP status: %d", *status_code);
    ESP_LOGI(TAG, "HTTP bytes: %d", *bytes_received);

    if (response.truncated) {
        ESP_LOGW(TAG, "HTTP response truncated to %d bytes", response.data_len);
    }

    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP failed: %s", esp_err_to_name(err));
        return false;
    }

    if (*status_code < 200 || *status_code >= 300) {
        ESP_LOGE(TAG, "HTTP bad status: %d", *status_code);
        return false;
    }

    return true;
}

// =========================================================
// JSON-LITE / GEO / API
// =========================================================

static bool parse_json_number(const char *json, const char *key, double *value)
{
    const char *found = strstr(json, key);
    if (!found) return false;

    found = strchr(found, ':');
    if (!found) return false;

    found++;

    while (*found == ' ' || *found == '\t') found++;

    char *endptr = NULL;
    double parsed = strtod(found, &endptr);

    if (endptr == found) return false;

    *value = parsed;
    return true;
}

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return p;
}

static const char *skip_json_value(const char *p)
{
    p = skip_ws(p);

    if (*p == '"') {
        p++;
        while (*p) {
            if (*p == '\\' && p[1]) p += 2;
            else if (*p == '"') {
                p++;
                break;
            } else p++;
        }
        return p;
    }

    if (*p == '[') {
        int depth = 0;
        while (*p) {
            if (*p == '[') depth++;
            else if (*p == ']') {
                depth--;
                if (depth == 0) {
                    p++;
                    break;
                }
            }
            p++;
        }
        return p;
    }

    if (*p == '{') {
        int depth = 0;
        while (*p) {
            if (*p == '{') depth++;
            else if (*p == '}') {
                depth--;
                if (depth == 0) {
                    p++;
                    break;
                }
            }
            p++;
        }
        return p;
    }

    while (*p && *p != ',' && *p != ']') p++;
    return p;
}

static void clean_callsign(char *dst, int dst_len, const char *src)
{
    int j = 0;

    if (!src) {
        snprintf(dst, dst_len, "NOID");
        return;
    }

    while (*src && j < dst_len - 1) {
        if (*src != ' ' && *src != '"') {
            dst[j++] = (char)toupper((unsigned char)*src);
        }
        src++;
    }

    dst[j] = '\0';

    if (j == 0) snprintf(dst, dst_len, "NOID");
}

static void populate_aircraft_geo(aircraft_t *out, double home_lat, double home_lon)
{
    double lat1 = home_lat * M_PI / 180.0;
    double lat2 = out->lat * M_PI / 180.0;
    double dlat = (out->lat - home_lat) * M_PI / 180.0;
    double dlon = (out->lon - home_lon) * M_PI / 180.0;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1) * cos(lat2) *
               sin(dlon / 2.0) * sin(dlon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    out->distance_km = 6371.0 * c;

    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) -
               sin(lat1) * cos(lat2) * cos(dlon);

    out->bearing_deg = atan2(y, x) * 180.0 / M_PI;

    while (out->bearing_deg < 0) out->bearing_deg += 360.0;
    while (out->bearing_deg >= 360.0) out->bearing_deg -= 360.0;

    out->valid = true;
}

static const char *find_json_object_key(const char *object_start, const char *key)
{
    const char *object_end = skip_json_value(object_start);
    char key_pattern[32];

    snprintf(key_pattern, sizeof(key_pattern), "\"%s\"", key);

    const char *found = strstr(object_start, key_pattern);

    while (found && found < object_end) {
        const char *colon = strchr(found, ':');

        if (colon && colon < object_end) {
            return skip_ws(colon + 1);
        }

        found = strstr(found + 1, key_pattern);
    }

    return NULL;
}

static bool read_json_object_number(const char *object_start, const char *key, double *value)
{
    const char *p = find_json_object_key(object_start, key);
    if (!p) return false;

    char *endptr = NULL;
    double parsed = strtod(p, &endptr);

    if (endptr == p) return false;

    *value = parsed;
    return true;
}

static bool read_json_object_text(const char *object_start, const char *key, char *out, int out_len)
{
    const char *p = find_json_object_key(object_start, key);
    if (!p || *p != '"') return false;

    p++;
    int j = 0;

    while (*p && *p != '"' && j < out_len - 1) {
        if (*p == '\\' && p[1]) p++;
        out[j++] = *p++;
    }

    out[j] = '\0';
    return j > 0;
}

static bool read_json_object_altitude_ft(const char *object_start, const char *key, double *altitude_ft)
{
    if (read_json_object_number(object_start, key, altitude_ft)) {
        return true;
    }

    char text[16] = {0};

    if (read_json_object_text(object_start, key, text, sizeof(text)) &&
        (strcmp(text, "ground") == 0 || strcmp(text, "GROUND") == 0)) {
        *altitude_ft = 0.0;
        return true;
    }

    return false;
}

static bool looks_like_airline_callsign(const char *callsign)
{
    int letters = 0;
    int digits = 0;

    for (int i = 0; callsign[i]; i++) {
        if (isalpha((unsigned char)callsign[i])) {
            letters++;
        } else if (isdigit((unsigned char)callsign[i])) {
            digits++;
        } else if (callsign[i] == '-') {
            return false;
        }
    }

    return letters >= 2 && letters <= 4 && digits >= 1;
}

static bool aircraft_type_is_airliner(const char *type_code, const char *description)
{
    static const char *airliner_prefixes[] = {
        "A3", "A2", "A1", "A4", "A5", "B7", "B6", "B3", "B4", "B5",
        "E17", "E19", "E29", "CRJ", "DH8", "AT7", "AT4", "SF3", "SB2"
    };

    for (int i = 0; i < (int)(sizeof(airliner_prefixes) / sizeof(airliner_prefixes[0])); i++) {
        size_t len = strlen(airliner_prefixes[i]);

        if (strncmp(type_code, airliner_prefixes[i], len) == 0) {
            return true;
        }
    }

    return strstr(description, "AIRBUS") ||
           strstr(description, "BOEING") ||
           strstr(description, "EMBRAER") ||
           strstr(description, "CANADAIR REGIONAL") ||
           strstr(description, "DE HAVILLAND CANADA DHC-8") ||
           strstr(description, "ATR ");
}

static bool id_starts_with(const char *id, const char *prefix)
{
    return strncmp(id, prefix, strlen(prefix)) == 0;
}

static bool id_is_police(const char *id)
{
    return strcmp(id, "GPOLA") == 0 ||
           id_starts_with(id, "GPOL") ||
           id_starts_with(id, "NPAS") ||
           id_starts_with(id, "POL") ||
           id_starts_with(id, "UKP");
}

static bool id_is_air_ambulance(const char *id)
{
    return strcmp(id, "GWASS") == 0 ||
           strcmp(id, "GHEMC") == 0 ||
           id_starts_with(id, "HELIMED") ||
           id_starts_with(id, "HLE") ||
           strstr(id, "AMB") != NULL;
}

static void classify_aircraft(const char *object_start, aircraft_t *out)
{
    double db_flags = 0.0;
    char type_code[16] = {0};
    char description[64] = {0};
    char registration_raw[32] = {0};
    char registration[16] = {0};

    if (read_json_object_text(object_start, "r", registration_raw, sizeof(registration_raw))) {
        clean_callsign(registration, sizeof(registration), registration_raw);
    }

    if (id_is_police(out->callsign) || id_is_police(registration)) {
        snprintf(out->aircraft_class, sizeof(out->aircraft_class), "POLICE");
        return;
    }

    if (id_is_air_ambulance(out->callsign) || id_is_air_ambulance(registration)) {
        snprintf(out->aircraft_class, sizeof(out->aircraft_class), "AIR AMB");
        return;
    }

    if (read_json_object_number(object_start, "dbFlags", &db_flags) &&
        (((int)db_flags) & 1)) {
        snprintf(out->aircraft_class, sizeof(out->aircraft_class), "MILITARY");
        return;
    }

    read_json_object_text(object_start, "t", type_code, sizeof(type_code));
    clean_callsign(type_code, sizeof(type_code), type_code);

    read_json_object_text(object_start, "desc", description, sizeof(description));
    clean_callsign(description, sizeof(description), description);

    if (looks_like_airline_callsign(out->callsign) ||
        aircraft_type_is_airliner(type_code, description)) {
        snprintf(out->aircraft_class, sizeof(out->aircraft_class), "COMMERCIAL");
    } else {
        snprintf(out->aircraft_class, sizeof(out->aircraft_class), "PRIVATE");
    }
}

static bool parse_single_airplanes_aircraft(const char *object_start,
                                            double home_lat,
                                            double home_lon,
                                            aircraft_t *out)
{
    memset(out, 0, sizeof(*out));

    if (*object_start != '{') return false;

    if (!read_json_object_number(object_start, "lat", &out->lat)) return false;
    if (!read_json_object_number(object_start, "lon", &out->lon)) return false;

    char ident[32] = {0};

    if (read_json_object_text(object_start, "flight", ident, sizeof(ident))) {
        clean_callsign(out->callsign, sizeof(out->callsign), ident);

        if (strcmp(out->callsign, "NOID") == 0 &&
            read_json_object_text(object_start, "r", ident, sizeof(ident))) {
            clean_callsign(out->callsign, sizeof(out->callsign), ident);
        }
    } else if (read_json_object_text(object_start, "r", ident, sizeof(ident)) ||
               read_json_object_text(object_start, "hex", ident, sizeof(ident))) {
        clean_callsign(out->callsign, sizeof(out->callsign), ident);
    } else {
        snprintf(out->callsign, sizeof(out->callsign), "NOID");
    }

    double altitude_ft = 0.0;

    if (read_json_object_altitude_ft(object_start, "alt_baro", &altitude_ft) ||
        read_json_object_altitude_ft(object_start, "alt_geom", &altitude_ft)) {
        out->altitude_m = altitude_ft * 0.3048;
    } else {
        out->altitude_m = -1;
    }

    double speed_kt = 0.0;

    if (read_json_object_number(object_start, "gs", &speed_kt)) {
        out->velocity_ms = speed_kt / 1.94384;
    } else {
        out->velocity_ms = -1;
    }

    if (!read_json_object_number(object_start, "track", &out->heading_deg) &&
        !read_json_object_number(object_start, "true_heading", &out->heading_deg) &&
        !read_json_object_number(object_start, "mag_heading", &out->heading_deg)) {
        out->heading_deg = -1;
    }

    classify_aircraft(object_start, out);
    populate_aircraft_geo(out, home_lat, home_lon);
    return true;
}

static bool parse_airplanes_live_aircraft(const char *json,
                                          double home_lat,
                                          double home_lon,
                                          aircraft_list_t *list)
{
    list->count = 0;

    const char *aircraft = strstr(json, "\"ac\"");
    if (!aircraft) {
        aircraft = strstr(json, "\"aircraft\"");
    }
    if (!aircraft) return true;

    const char *p = strchr(aircraft, '[');
    if (!p) return true;

    p++;

    while (*p && list->count < MAX_AIRCRAFT) {
        p = skip_ws(p);

        if (*p == ']') break;

        if (*p == '{') {
            aircraft_t candidate;

            if (parse_single_airplanes_aircraft(p, home_lat, home_lon, &candidate)) {
                if (candidate.distance_km <= g_radar_range_km) {
                    list->aircraft[list->count++] = candidate;
                }
            }

            p = skip_json_value(p);
        } else {
            p++;
        }

        p = skip_ws(p);
        if (*p == ',') p++;
    }

    return true;
}

static bool postcode_lookup(double *latitude, double *longitude)
{
    char *response_buffer = calloc(1, HTTP_RESPONSE_BUFFER_SIZE);

    if (!response_buffer) {
        ESP_LOGE(TAG, "No RAM for postcode HTTP buffer");
        return false;
    }

    int status_code = 0;
    int bytes = 0;

    char postcode_url[128];
    snprintf(postcode_url, sizeof(postcode_url), "http://api.postcodes.io/postcodes/%s", g_postcode_encoded);

    bool ok = http_get(postcode_url, response_buffer, HTTP_RESPONSE_BUFFER_SIZE, NULL, &status_code, &bytes);

    if (!ok) {
        free(response_buffer);
        return false;
    }

    bool got_lat = parse_json_number(response_buffer, "\"latitude\"", latitude);
    bool got_lon = parse_json_number(response_buffer, "\"longitude\"", longitude);

    if (!got_lat || !got_lon) {
        ESP_LOGE(TAG, "Failed to parse postcode latitude/longitude");
        free(response_buffer);
        return false;
    }

    ESP_LOGI(TAG, "Postcode resolved: %.6f %.6f", *latitude, *longitude);

    free(response_buffer);
    return true;
}

static bool airplanes_live_fetch_aircraft(double home_lat, double home_lon, aircraft_list_t *list)
{
    double radius_nm = g_radar_range_miles * 0.868976;

    if (radius_nm < 1.0) {
        radius_nm = 1.0;
    }

    if (radius_nm > 250.0) {
        radius_nm = 250.0;
    }

    char url[360];

    snprintf(url,
             sizeof(url),
             "https://api.airplanes.live/v2/point/%.6f/%.6f/%.1f",
             home_lat,
             home_lon,
             radius_nm);

    char *response_buffer = calloc(1, AIRCRAFT_RESPONSE_BUFFER_SIZE);

    if (!response_buffer) {
        ESP_LOGE(TAG, "No RAM for Airplanes.live HTTP buffer");
        return false;
    }

    int status_code = 0;
    int bytes = 0;

    bool ok = http_get(url,
                       response_buffer,
                       AIRCRAFT_RESPONSE_BUFFER_SIZE,
                       AIRPLANES_LIVE_ROOT_CA,
                       &status_code,
                       &bytes);

    if (!ok) {
        free(response_buffer);
        return false;
    }

    parse_airplanes_live_aircraft(response_buffer, home_lat, home_lon, list);

    ESP_LOGI(TAG, "Airplanes.live aircraft parsed: %d", list->count);

    for (int i = 0; i < list->count; i++) {
        ESP_LOGI(TAG,
                 "%s dist %.1f km bearing %.0f alt %.0f",
                 list->aircraft[i].callsign,
                 list->aircraft[i].distance_km,
                 list->aircraft[i].bearing_deg,
                 list->aircraft[i].altitude_m);
    }

    free(response_buffer);
    return true;
}

// =========================================================
// UI / RADAR
// =========================================================

static void show_status_screen(uint16_t bg, const char *line1, const char *line2)
{
    lcd_fill(bg);

    if (line2 && strlen(line2) > 0) {
        int y = (LCD_HEIGHT - ((2 - 1) * 32 + 14)) / 2;
        lcd_draw_text_centered(y, line1, WHITE, 2);
        lcd_draw_text_centered(y + 32, line2, WHITE, 2);
    } else {
        lcd_draw_text_centered((LCD_HEIGHT - 14) / 2, line1, WHITE, 2);
    }
}

static void show_status_screen3_colour(uint16_t bg,
                                       const char *line1,
                                       const char *line2,
                                       const char *line3,
                                       uint16_t text_colour)
{
    int y = (LCD_HEIGHT - ((3 - 1) * 32 + 14)) / 2;

    lcd_fill(bg);
    lcd_draw_text_centered(y, line1, text_colour, 2);
    lcd_draw_text_centered(y + 32, line2, text_colour, 2);
    lcd_draw_text_centered(y + 64, line3, text_colour, 2);
}

static void show_status_screen2_colour(uint16_t bg,
                                       const char *line1,
                                       const char *line2,
                                       uint16_t text_colour)
{
    int y = (LCD_HEIGHT - ((2 - 1) * 32 + 14)) / 2;

    lcd_fill(bg);
    lcd_draw_text_centered(y, line1, text_colour, 2);
    lcd_draw_text_centered(y + 32, line2, text_colour, 2);
}

static void show_hold_config_prompt(bool registered)
{
    uint16_t colour = registered ? GREEN : YELLOW;

    lcd_fill(BLACK);
    lcd_draw_circle(RADAR_CX, RADAR_CY, 78, colour);
    lcd_draw_circle(RADAR_CX, RADAR_CY, 79, colour);
    lcd_draw_circle(RADAR_CX, RADAR_CY, 80, colour);

    if (registered) {
        int y = (LCD_HEIGHT - ((2 - 1) * 32 + 14)) / 2;
        lcd_draw_text_centered(y, "ENTERING", colour, 2);
        lcd_draw_text_centered(y + 32, "CONFIG", colour, 2);
    } else {
        int y = (LCD_HEIGHT - ((3 - 1) * 32 + 14)) / 2;
        lcd_draw_text_centered(y, "HOLD", colour, 2);
        lcd_draw_text_centered(y + 32, "FOR", colour, 2);
        lcd_draw_text_centered(y + 64, "CONFIG", colour, 2);
    }
}

static void compute_aircraft_screen_position(aircraft_t *a)
{
    double range_fraction = a->distance_km / g_radar_range_km;

    if (range_fraction < 0.0) range_fraction = 0.0;
    if (range_fraction > 1.0) range_fraction = 1.0;

    double bearing_rad = a->bearing_deg * M_PI / 180.0;
    double r = range_fraction * RADAR_RADIUS;

    a->screen_x = RADAR_CX + (int)round(sin(bearing_rad) * r);
    a->screen_y = RADAR_CY - (int)round(cos(bearing_rad) * r);
}

static uint16_t aircraft_colour(const aircraft_t *a)
{
    if (strcmp(a->aircraft_class, "COMMERCIAL") == 0) {
        return CYAN;
    }

    if (strcmp(a->aircraft_class, "MILITARY") == 0) {
        return GREEN;
    }

    if (strcmp(a->aircraft_class, "POLICE") == 0) {
        return BLUE;
    }

    if (strcmp(a->aircraft_class, "AIR AMB") == 0) {
        return MAGENTA;
    }

    return YELLOW;
}

static void aircraft_update_task(void *arg)
{
    (void)arg;

    while (1) {
        aircraft_list_t fetched = {0};
        bool ok = airplanes_live_fetch_aircraft(g_home_lat, g_home_lon, &fetched);

        if (ok) {
            for (int i = 0; i < fetched.count; i++) {
                compute_aircraft_screen_position(&fetched.aircraft[i]);
            }
        }

        if (g_aircraft_mutex && xSemaphoreTake(g_aircraft_mutex, portMAX_DELAY) == pdTRUE) {
            if (ok) {
                g_aircraft_latest = fetched;
            }

            g_aircraft_latest_ok = ok;
            g_aircraft_has_update = true;
            g_aircraft_update_seq++;
            xSemaphoreGive(g_aircraft_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(AIRCRAFT_UPDATE_INTERVAL_MS));
    }
}

static void draw_radar_base(double sweep_angle_deg)
{
    radar_fb_clear(BLACK);

    // Dynamic range rings:
    // one ring every 5 miles, with the outer ring representing the configured range.
    // Examples:
    // 10 miles = 5 mile inner ring + 10 mile outer ring
    // 20 miles = 5, 10, 15 mile rings + 20 mile outer ring
    int max_range_miles = (int)round(g_radar_range_miles);

    if (max_range_miles < 1) {
        max_range_miles = 1;
    }

    int ring_step_miles = 5;

    for (int miles = ring_step_miles; miles < max_range_miles; miles += ring_step_miles) {
        int r = (int)round(((double)miles / g_radar_range_miles) * RADAR_RADIUS);

        if (r > 2 && r < RADAR_RADIUS) {
            radar_fb_draw_circle(RADAR_CX, RADAR_CY, r, DIMGREEN);
        }
    }

    // Always draw the configured maximum range as the outer ring.
    radar_fb_draw_circle(RADAR_CX, RADAR_CY, RADAR_RADIUS, YELLOW);

    radar_fb_draw_line(RADAR_CX - RADAR_RADIUS, RADAR_CY, RADAR_CX + RADAR_RADIUS, RADAR_CY, DIMGREEN);
    radar_fb_draw_line(RADAR_CX, RADAR_CY - RADAR_RADIUS, RADAR_CX, RADAR_CY + RADAR_RADIUS, DIMGREEN);

    radar_fb_fill_circle(RADAR_CX, RADAR_CY, 3, YELLOW);

    char title_line[24];
    snprintf(title_line, sizeof(title_line), "%.0f MILE", g_radar_range_miles);
    radar_fb_draw_text_centered(8, title_line, YELLOW, 1);

    radar_fb_draw_sweep(sweep_angle_deg);

    // North-up indicator. The radar is map-style north-up, not device-heading-up.
    radar_fb_draw_text_centered(21, "N", WHITE, 2);

    radar_fb_fill_circle(21, 120, 16, YELLOW);
    radar_fb_draw_circle(21, 120, 16, BLACK);
    radar_fb_draw_text(16, 113, "-", BLACK, 2);

    radar_fb_fill_circle(219, 120, 16, YELLOW);
    radar_fb_draw_circle(219, 120, 16, BLACK);
    radar_fb_draw_text(214, 113, "+", BLACK, 2);

    radar_fb_draw_text(8, 224, g_postcode, YELLOW, 1);
}

static void draw_aircraft_marker(aircraft_t *a)
{
    if (a->distance_km > g_radar_range_km) {
        return;
    }

    compute_aircraft_screen_position(a);

    int x = a->screen_x;
    int y = a->screen_y;
    uint16_t marker_colour = aircraft_colour(a);

    radar_fb_fill_circle(x, y, 3, marker_colour);

    if (a->heading_deg >= 0.0) {
        double speed_kt = a->velocity_ms >= 0.0 ? a->velocity_ms * 1.94384 : 0.0;
        int nose_len = 5 + (int)round(speed_kt / 25.0);

        if (nose_len < 5) nose_len = 5;
        if (nose_len > 18) nose_len = 18;

        double heading_rad = a->heading_deg * M_PI / 180.0;
        int nose_x = x + (int)round(sin(heading_rad) * nose_len);
        int nose_y = y - (int)round(cos(heading_rad) * nose_len);
        radar_fb_draw_line(x, y, nose_x, nose_y, marker_colour);
    }

    if (a->distance_km < 8.0) {
        int label_x = x + 5;
        int label_y = y - 4;

        if (label_x > 178) label_x = x - 48;
        if (label_y < 18) label_y = y + 8;
        if (label_y > 210) label_y = y - 12;

        radar_fb_draw_text(label_x, label_y, a->callsign, marker_colour, 1);
    }
}

static void draw_radar_screen(aircraft_list_t *list, double sweep_angle_deg)
{
    draw_radar_base(sweep_angle_deg);

    for (int i = 0; i < list->count; i++) {
        if (list->aircraft[i].distance_km <= g_radar_range_km) {
            draw_aircraft_marker(&list->aircraft[i]);
        }
    }

    const aircraft_t *nearest = NULL;

    for (int i = 0; i < list->count; i++) {
        if (list->aircraft[i].distance_km <= g_radar_range_km) {
            if (!nearest || list->aircraft[i].distance_km < nearest->distance_km) {
                nearest = &list->aircraft[i];
            }
        }
    }

    if (nearest) {
        char nearest_line[32];
        snprintf(nearest_line, sizeof(nearest_line), "NEAR %.1fMI", nearest->distance_km * 0.621371);
        radar_fb_draw_text_centered(204, nearest_line, YELLOW, 1);
    } else {
        radar_fb_draw_text_centered(204, "NO LOCAL AC", YELLOW, 1);
    }

    lcd_blit_radar_frame();
}

static int find_aircraft_at_touch(const aircraft_list_t *list, int tx, int ty)
{
    int best_index = -1;
    int best_dist_sq = TOUCH_SELECT_RADIUS_PX * TOUCH_SELECT_RADIUS_PX;

    for (int i = 0; i < list->count; i++) {
        if (list->aircraft[i].distance_km > g_radar_range_km) {
            continue;
        }

        int dx = tx - list->aircraft[i].screen_x;
        int dy = ty - list->aircraft[i].screen_y;
        int dist_sq = (dx * dx) + (dy * dy);

        if (dist_sq <= best_dist_sq) {
            best_dist_sq = dist_sq;
            best_index = i;
        }
    }

    return best_index;
}

static int touch_range_direction(int tx, int ty)
{
    if (ty < RANGE_TOUCH_Y_MIN || ty > RANGE_TOUCH_Y_MAX) {
        return 0;
    }

    if (tx >= RANGE_TOUCH_PLUS_X_MIN && tx <= RANGE_TOUCH_PLUS_X_MAX) {
        return 1;
    }

    if (tx >= RANGE_TOUCH_MINUS_X_MIN && tx <= RANGE_TOUCH_MINUS_X_MAX) {
        return -1;
    }

    return 0;
}

static void apply_range_change(int direction)
{
    change_radar_range(direction);
}

static void draw_aircraft_detail(const aircraft_t *a)
{
    char line[40];

    lcd_fill(BLACK);

    lcd_draw_text_centered(14, "AIRCRAFT", YELLOW, 2);
    lcd_draw_text_centered(44, a->callsign, CYAN, 3);

    double distance_miles = a->distance_km * 0.621371;
    snprintf(line, sizeof(line), "DIST %.1f MI", distance_miles);
    lcd_draw_text_centered(94, line, WHITE, 2);

    if (a->velocity_ms >= 0.0) {
        int speed_mph = (int)round(a->velocity_ms * 2.23694);
        snprintf(line, sizeof(line), "%d MPH", speed_mph);
    } else {
        snprintf(line, sizeof(line), "MPH UNKNOWN");
    }
    lcd_draw_text_centered(126, line, WHITE, 2);

    if (a->altitude_m >= 0.0) {
        int altitude_ft = (int)round(a->altitude_m * 3.28084);
        char altitude_text[16];
        format_int_with_commas(altitude_ft, altitude_text, sizeof(altitude_text));
        snprintf(line, sizeof(line), "ALT %s FT", altitude_text);
    } else {
        snprintf(line, sizeof(line), "ALT UNKNOWN");
    }
    lcd_draw_text_centered(158, line, WHITE, 2);

    lcd_draw_text_centered(188, a->aircraft_class, aircraft_colour(a), 2);

    lcd_draw_text_centered(216, "TAP TO RETURN", YELLOW, 1);
}

static bool boot_config_touch_requested(void)
{
    if (active_touch_addr == 0) {
        return false;
    }

    show_hold_config_prompt(false);

    int64_t start_ms = esp_timer_get_time() / 1000;

    while ((esp_timer_get_time() / 1000) - start_ms < 2000) {
        int raw_x = 0;
        int raw_y = 0;
        int gesture = 0;
        int fingers = 0;

        if (touch_read_raw(&raw_x, &raw_y, &gesture, &fingers)) {
            ESP_LOGW(TAG, "Boot touch detected. Starting setup portal.");
            show_hold_config_prompt(true);
            vTaskDelay(pdMS_TO_TICKS(350));
            show_status_screen(DARK, "CONFIG", "MODE");
            vTaskDelay(pdMS_TO_TICKS(700));
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return false;
}

// =========================================================
// MAIN
// =========================================================

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 aircraft radar direct-draw v2 starting");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    lcd_init_hardware();

    show_status_screen(BLUE, "BOOT", "OK");
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool saved_config = settings_load();

    bool touch_ok = touch_init();

    if (touch_ok) {
        ESP_LOGI(TAG, "Touch ready at 0x%02X", active_touch_addr);
    } else {
        ESP_LOGW(TAG, "Touch not available; radar still works");
    }

    // Hold/touch the screen during this short boot window to re-enter setup mode.
    if (touch_ok && boot_config_touch_requested()) {
        start_config_portal();
    }

    if (!saved_config) {
        ESP_LOGW(TAG, "No saved config found. Starting setup portal.");
        start_config_portal();
    }

    show_status_screen(DARK, "WIFI", "START");

    bool connected = wifi_connect();

    if (!connected) {
        show_status_screen(RED, "WIFI", "FAIL");
        ESP_LOGE(TAG, "Wi-Fi failed. Starting setup portal.");
        vTaskDelay(pdMS_TO_TICKS(1000));
        start_config_portal();
    }

    show_status_screen2_colour(GREEN, "WIFI", "OK", BLACK);
    ESP_LOGI(TAG, "Wi-Fi OK");

    vTaskDelay(pdMS_TO_TICKS(1000));

    show_status_screen(DARK, "POSTCODE", "SEARCH");

    double home_lat = 0.0;
    double home_lon = 0.0;

    if (!postcode_lookup(&home_lat, &home_lon)) {
        show_status_screen(RED, "POST", "FAIL");
        ESP_LOGE(TAG, "Postcode lookup failed");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    show_status_screen3_colour(GREEN, "POSTCODE", "FOUND", "OK", BLACK);
    vTaskDelay(pdMS_TO_TICKS(1000));

    g_home_lat = home_lat;
    g_home_lon = home_lon;
    g_aircraft_mutex = xSemaphoreCreateMutex();

    if (!g_aircraft_mutex) {
        show_status_screen(RED, "OPEN", "FAIL");
        ESP_LOGE(TAG, "Failed to create aircraft update mutex");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    show_status_screen(DARK, "REFRESHING", "");

    if (xTaskCreate(aircraft_update_task, "aircraft_update", 8192, NULL, 5, NULL) != pdPASS) {
        show_status_screen(RED, "OPEN", "FAIL");
        ESP_LOGE(TAG, "Failed to start aircraft update task");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    aircraft_list_t list = {0};
    uint32_t seen_aircraft_update_seq = 0;
    bool showing_detail = false;
    bool radar_visible = false;
    double sweep_angle_deg = 0.0;
    int64_t last_sweep_ms = 0;
    int64_t detail_started_ms = 0;

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        bool have_aircraft_update = false;
        bool aircraft_update_ok = false;

        if (g_aircraft_mutex && xSemaphoreTake(g_aircraft_mutex, 0) == pdTRUE) {
            if (g_aircraft_has_update && g_aircraft_update_seq != seen_aircraft_update_seq) {
                seen_aircraft_update_seq = g_aircraft_update_seq;
                aircraft_update_ok = g_aircraft_latest_ok;
                have_aircraft_update = true;

                if (aircraft_update_ok) {
                    list = g_aircraft_latest;
                }
            }

            xSemaphoreGive(g_aircraft_mutex);
        }

        if (have_aircraft_update && !showing_detail) {
            if (aircraft_update_ok) {
                draw_radar_screen(&list, sweep_angle_deg);
                radar_visible = true;
                last_sweep_ms = now_ms;
            } else {
                ESP_LOGW(TAG, "Airplanes.live lookup failed; keeping last radar data");

                if (list.count == 0 && !radar_visible) {
                    draw_radar_screen(&list, sweep_angle_deg);
                    radar_visible = true;
                    last_sweep_ms = now_ms;
                }
            }
        }

        if (radar_visible && !showing_detail && now_ms - last_sweep_ms >= RADAR_SWEEP_INTERVAL_MS) {
            sweep_angle_deg += RADAR_SWEEP_STEP_DEG;
            if (sweep_angle_deg >= 360.0) {
                sweep_angle_deg -= 360.0;
            }

            draw_radar_screen(&list, sweep_angle_deg);
            last_sweep_ms = now_ms;
        }

        if (touch_ok) {
            int raw_x = 0;
            int raw_y = 0;
            int gesture = 0;
            int fingers = 0;

            if (touch_read_raw(&raw_x, &raw_y, &gesture, &fingers)) {
                int sx = 0;
                int sy = 0;
                map_touch_to_screen(raw_x, raw_y, &sx, &sy);

                ESP_LOGI(TAG, "Touch x=%d y=%d gesture=%d fingers=%d", sx, sy, gesture, fingers);

                if (showing_detail) {
                    showing_detail = false;
                    draw_radar_screen(&list, sweep_angle_deg);
                    radar_visible = true;
                    last_sweep_ms = now_ms;
                    vTaskDelay(pdMS_TO_TICKS(500));
                } else {
                    int range_direction = touch_range_direction(sx, sy);

                    if (range_direction != 0) {
                        ESP_LOGI(TAG, "Range touch button pressed: %s", range_direction > 0 ? "plus" : "minus");
                        apply_range_change(range_direction);
                        vTaskDelay(pdMS_TO_TICKS(350));
                    } else {
                        int selected = find_aircraft_at_touch(&list, sx, sy);

                        if (selected >= 0) {
                            ESP_LOGI(TAG, "Selected aircraft %s", list.aircraft[selected].callsign);
                            draw_aircraft_detail(&list.aircraft[selected]);
                            showing_detail = true;
                            detail_started_ms = now_ms;
                            vTaskDelay(pdMS_TO_TICKS(500));
                        } else {
                            // Missed tap: do nothing.
                            // The older version drew a magenta touch cross here, but on the round LCD
                            // it could leave tiny coloured artefacts near the edge of the radar.
                            vTaskDelay(pdMS_TO_TICKS(250));
                        }
                    }
                }
            }
        }

        if (showing_detail) {
            now_ms = esp_timer_get_time() / 1000;
            if (now_ms - detail_started_ms > 10000) {
                showing_detail = false;
                draw_radar_screen(&list, sweep_angle_deg);
                radar_visible = true;
                last_sweep_ms = now_ms;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
