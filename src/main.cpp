#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <driver/i2s.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>


void myAudioMethod (void * pvParameters);
void myAnalyzeMethod (void * pvParameters);

// Audio stuff
const i2s_port_t I2S_PORT = I2S_NUM_0;
const uint16_t sample_count = 64;
uint16_t sample_counter = 0;
const int sampling_frequency = 16000; // Hz
unsigned int sampling_period_us;


long last_sample = -1;
double samples_real[sample_count];
double samples_imaginary[sample_count];
double samples_real_fft[sample_count];
double samples_imaginary_fft[sample_count];

bool calculate_fft_now = false;

arduinoFFT FFT = arduinoFFT();

//OLED pins
#define OLED_SDA 21
#define OLED_SCL 22 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
    // OLED stuff
    if(!oledDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 nicht gefunden");
        for(;;);
    }
    oledDisplay.clearDisplay();  // Display(puffer) löschen
    oledDisplay.setTextSize(1);  // kleine Schriftgröße (Höhe 8px)
    oledDisplay.setTextColor(WHITE);  // helle Schrift, dunkler Grund)
    oledDisplay.display();

    // I2S Audio init stuff
    Serial.begin(115200);
    esp_err_t err;
    sampling_period_us = round(1000000*(1.0/sampling_frequency));

    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,                         // 16KHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
        .dma_buf_count = 4,                           // number of buffers
        .dma_buf_len = 1024                           // 8 samples per buffer (minimum)
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

    // Multicore stuff
    TaskHandle_t Task1;
    TaskHandle_t Task2;
    pinMode(25, OUTPUT);

    xTaskCreatePinnedToCore(
        myAudioMethod,
        "Task1",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        10,           /* priority of the task */
        &Task1,      /* Task handle to keep track of created task */
        0            /* pin task to core 0 */
    );
    delay(500); 

    xTaskCreatePinnedToCore(
        myAnalyzeMethod,
        "Task2",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        10,           /* priority of the task */
        &Task2,      /* Task handle to keep track of created task */
        1            /* pin task to core 1 */
    );
    delay(500);
}

void myAudioMethod( void * pvParameters ){
    unsigned long microseconds = micros();

    while(true) {
        // Read a single sample and log it for the Serial Plotter.
        int32_t sample = 0;
        //int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&sample, portMAX_DELAY); // no timeout
        size_t bytes_read;
        i2s_read(I2S_PORT, (char *)&sample, 4, &bytes_read, portMAX_DELAY);
        if (bytes_read <= 0) {
            Serial.println("no samples read.");
            while(true);   
        }

        //Serial.println(sample);
        last_sample = bytes_read;

        samples_real[sample_counter] = sample;
        samples_imaginary[sample_counter] = 0;
        
        sample_counter++;
        if(sample_counter == 64) {
            sample_counter = 0;
            calculate_fft_now = true;
        }
        
        // wait to match sample frequency
        while(micros() - microseconds < sampling_period_us){
        }
        microseconds += sampling_period_us;

        vTaskDelay(1);
    }
}


void copy_samples() {
    for(int i=0; i < sample_count; i++) {
        samples_real_fft[i] = samples_real[i];
        samples_imaginary_fft[i] = samples_imaginary[i];
    }
}


void print_array(double array[]) {
    for(int i = 0; i < sample_count; i++) {
        Serial.println(array[i] / 1000);
        //Serial.print(" ");
    }
    Serial.println();
}



void myAnalyzeMethod( void * pvParameters ){
    /*Serial.print("Task2 running on core ");
    Serial.println(xPortGetCoreID());*/

    while(true) {
        oledDisplay.clearDisplay();  // Display(puffer) löschen
        oledDisplay.setCursor(0, 0);  // links oben anfangen
        
        if(calculate_fft_now) {
            calculate_fft_now = false;
        } else {
            continue;
        }
        // copy samples into safely mutable arrays
        copy_samples();

        Serial.println("samples before:");
        print_array(samples_real_fft);
        
        // weigh data with Hamming weighting
        FFT.Windowing(samples_real_fft, sample_count, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

        // compute FFT
        FFT.Compute(samples_real_fft, samples_imaginary_fft, sample_count, FFT_FORWARD);

        // to magnitudes
        FFT.ComplexToMagnitude(samples_real_fft, samples_imaginary_fft, sample_count);

        Serial.println("samples after:");
        print_array(samples_real_fft);
        //print_array(samples_imaginary_fft);
        //Serial.println("---");

        //oledDisplay.println(samples_real_fft[0]);
        //oledDisplay.display();
        
        vTaskDelay(1);
    }
}




void loop() {
    vTaskDelete(NULL);
}