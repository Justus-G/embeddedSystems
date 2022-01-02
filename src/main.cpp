#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <driver/i2s.h>
#include "driver/i2s.h"
const i2s_port_t I2S_PORT = I2S_NUM_0;

void setup() {
    Serial.begin(115200);
    esp_err_t err;

    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,                         // 16KHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
        .dma_buf_count = 4,                           // number of buffers
        .dma_buf_len = 8                              // 8 samples per buffer (minimum)
    };

    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // Serial Clock (SCK)
        .ws_io_num = 13,    // Word Select (WS)
        .data_out_num = I2S_PIN_NO_CHANGE, // not used (only for speakers)
        .data_in_num = 12   // Serial Data (SD)
    };

    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true);
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true);
    }
}

void loop() {
    // Read a single sample and log it for the Serial Plotter.
    int32_t sample = 0;
    int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&sample, portMAX_DELAY); // no timeout
    if (bytes_read > 0) {
        Serial.println(sample);
    } else {
        Serial.println("no samples read.");
    }
}