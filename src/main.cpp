#include <Arduino.h>
/*#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>*/
#include <driver/i2s.h>

/*//OLED pins
#define OLED_SDA 21
#define OLED_SCL 22 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);*/



const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 32;

void setup()
{
    Serial.begin(115200);

    /*//initialize OLED
    if(!oledDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 nicht gefunden");
        for(;;);
    }

    oledDisplay.clearDisplay();  // Display(puffer) löschen
    oledDisplay.setTextSize(1);  // kleine Schriftgröße (Höhe 8px)
    oledDisplay.setTextColor(WHITE);  // helle Schrift, dunkler Grund)
    oledDisplay.setCursor(0, 0);  // links oben anfangen
    oledDisplay.println("SSD1306 allocated.");
    oledDisplay.display();*/

    Serial.println("Configuring I2S...");
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
        .sample_rate = 16000,                         // 16KHz
        // 8 bits -> error (invalid bits per sample)
        // 16 bits -> nur -1
        // 24 bits -> nur -256
        // 32 bits -> nur -1
        .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT, // could only get it to work with 32bits
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // although the SEL config should be left, it seems to transmit on right
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
        .dma_buf_count = 8,                           // number of buffers
        .dma_buf_len = BLOCK_SIZE                     // samples per buffer
    };

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 19,   // BCKL
        .ws_io_num = 26,    // LRCL
        .data_out_num = -1, // not used (only for speakers)
        .data_in_num = 23   // DOUT
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
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
    Serial.println("I2S driver installed.");
}

bool outputted = false;

void loop()
{
    // Read multiple samples at once and calculate the sound pressure
    int32_t samples[BLOCK_SIZE];

    size_t num_bytes_read;

    esp_err_t err = i2s_read(I2S_PORT, 
                            (char *)samples, 
                            BLOCK_SIZE,     // the doc says bytes, but its elements.
                            &num_bytes_read,
                            portMAX_DELAY); // no timeout
    
    if (err != ESP_OK) {
        Serial.printf("Failed reading from mic: %d\n", err);
        while (true);
    }

    int samples_read = num_bytes_read / 8;
    for (int i = 0; i < samples_read; ++i) {
        Serial.printf("%d", samples[i]);
        if(i != (samples_read - 1))
            Serial.print(", ");
    }
    Serial.println();
    

    /*int samples_read = num_bytes_read / 8;
    if (samples_read > 0) {

        float mean = 0;
        for (int i = 0; i < samples_read; ++i) {
            mean += (samples[i] >> 14);
        }
        mean /= samples_read;

        float maxsample = -1e8, minsample = 1e8;
        for (int i = 0; i < samples_read; ++i) {
            minsample = min(minsample, samples[i] - mean);
            maxsample = max(maxsample, samples[i] - mean);
        }
        Serial.println(maxsample - minsample);
    }*/

    // Read multiple samples at once and calculate the sound pressure
    /*int32_t samples[BLOCK_SIZE];
    int num_bytes_read = i2s_read_bytes(I2S_PORT, (char *)samples, 
                                        BLOCK_SIZE,     // the doc says bytes, but its elements.
                                        portMAX_DELAY); // no timeout
    
    int samples_read = num_bytes_read / 8;
    if (samples_read > 0) {

        float mean = 0;
        for (int i = 0; i < samples_read; ++i) {
            mean += samples[i];
        }
        Serial.println(mean);
    }*/

    //delay(300);
}