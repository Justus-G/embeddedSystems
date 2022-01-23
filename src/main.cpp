#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <driver/i2s.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>


unsigned long start_time;

// I2S stuff
QueueHandle_t pI2S_Queue_;                  // i2s event queue
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int sample_count = 512;               // sample count of each i2s reading
const int i2s_sampling_rate = 44100;        // Hz
const int i2s_buffer_size = 128;            // size of i2s DMA buffers
const int i2s_buffer_count = 4;             // count of i2s DMA buffers
const int i2s_read_size = 4 * sample_count; // 4 bytes per sample * #samples
int32_t samples[sample_count] = {0};        // destination of each i2s reading

// FFT stuff
double fft_imag[sample_count] = {};         // FFT imaginary input values (all 0)
double fft_real[sample_count] = {};         // FFT real input values are copied from samples[] before each FFT analysis
arduinoFFT FFT = arduinoFFT(fft_real, fft_imag, sample_count, i2s_sampling_rate);
const int fft_frequency_bin_count = 16;     // indices:                    0     1     2     3     4     5     6      7      8      9     10     11     12     13      14      15
const int16_t fft_frequency_bins[fft_frequency_bin_count]              = {100,  140,  196,  274,  384,  538,  753,  1054,  1476,  2066,  2893,  4050,  5669,  7937,  11112,  15557}; // Hz
const int16_t fft_frequency_bin_lower_indices[fft_frequency_bin_count] = {0,    1,    2,    3,    4,    5,    8,    11,    15,    21,    29,    40,    56,    79,    111,    155};
const int16_t fft_frequency_bin_upper_indices[fft_frequency_bin_count] = {  1,    2,    3,    4,    5,    8,   11,    15,    21,    29,    40,    56,    79,   111,    155,    256};
const float_t fft_frequency_bin_multiplier[fft_frequency_bin_count] =    {0.001,0.005, 0.3, 0.5,  0.7,  0.9,  1.0,   1.0,   1.0,   1.0,   1.0,   1.0,   1.0,   1.0,    1.0,    1.0};

// parallelization stuff
bool i2s_read_done = false;                 // core synchronization
void read_i2s(void * pvParameters);         // i2s read method
void analyze_data(void * pvParameters);     // FFT analyze method

// OLED Display stuff
#define OLED_SDA 21                         // SDA pin
#define OLED_SCL 22                         // SCL pin
#define SCREEN_WIDTH 128                    // OLED display width, in pixels
#define SCREEN_HEIGHT 32                    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C                 // 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
    Serial.begin(115200);
    esp_err_t err;

    // OLED stuff
    if(!oledDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("error initializing SSD1306 display.");
        while(true);
    }
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);         // small font size (height 8px)
    oledDisplay.setTextColor(WHITE);
    oledDisplay.println("initializing...");
    oledDisplay.display();

    // I2S Audio stuff
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = i2s_sampling_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
        .dma_buf_count = i2s_buffer_count,
        .dma_buf_len = i2s_buffer_size
    };

    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // Serial Clock (SCK)
        .ws_io_num = 13,    // Word Select (WS)
        .data_out_num = I2S_PIN_NO_CHANGE, // not used (only for speakers)
        .data_in_num = 12   // Serial Data (SD)
    };

    err = i2s_driver_install(I2S_PORT, &i2s_config, 16, &pI2S_Queue_);
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

    xTaskCreatePinnedToCore(
        read_i2s,
        "read_i2s",  // task name
        10000,       // task stack size
        NULL,        // task params
        10,          // task priority
        &Task1,      // task handle
        0            // core #
    );
    delay(500); 

    xTaskCreatePinnedToCore(
        analyze_data,
        "analyze_data", // task name
        10000,          // task stack size
        NULL,           // task params
        10,             // task priority
        &Task2,         // task handle
        1               // core #
    );
    delay(500);

    start_time = millis();
}


void check_i2s_speed() {
    i2s_event_t i2sEvent = {};
    uint16_t i2sEventRxDoneCount = 0;
    uint16_t i2sMsgCount = uxQueueMessagesWaiting(pI2S_Queue_);

    log_v("Number of I2S events waiting in queue: %d", i2sMsgCount);

    // Iterate over all events in the i2s event queue
    for (int i = 0; i < i2sMsgCount; i++){
        // Take next event from queue
        if ( xQueueReceive(pI2S_Queue_, (void*) &i2sEvent, 0) == pdTRUE ){
            switch (i2sEvent.type){
                case I2S_EVENT_DMA_ERROR:
                    log_e("I2S_EVENT_DMA_ERROR");
                    break;
                case I2S_EVENT_TX_DONE:
                    log_v("I2S_EVENT_TX_DONE");
                    break;
                case I2S_EVENT_RX_DONE:
                    log_v("I2S_EVENT_RX_DONE");
                    i2sEventRxDoneCount++;
                    break;
                case I2S_EVENT_MAX:
                    log_w("I2S_EVENT_MAX");
                    break;
            }
        }
    }

    // If there are more RX done events in the queue than expected, probably data processing takes too long
    if (i2sEventRxDoneCount > (sample_count / i2s_buffer_size)){
        log_w("Frame loss. Number of I2S_EVENT_RX_DONE events is: %d", i2sEventRxDoneCount);
        Serial.printf("Frame loss. Number of I2S_EVENT_RX_DONE events is: %d\n", i2sEventRxDoneCount);
        if(i2sEventRxDoneCount == 16) {
            // only stop program if there is no chance of catching up
            while(true);
        }
    }
    if (i2sEventRxDoneCount < (sample_count / i2s_buffer_size)){
        log_e("Configuration error? Number of I2S_EVENT_RX_DONE events is: %d", i2sEventRxDoneCount);
        Serial.printf("Configuration error? Number of I2S_EVENT_RX_DONE events is: %d\n", i2sEventRxDoneCount);
        while(true);
    }
}

// read takes 11000µs = 11ms
void read_i2s(void * pvParameters) {
    size_t bytes_read;
    esp_err_t err;

    unsigned long time;

    while(true) {
        err = i2s_read(I2S_PORT, &samples, i2s_read_size, &bytes_read, portMAX_DELAY);
        if(bytes_read != i2s_read_size) {
            Serial.printf("read incorrect amount of samples: Expected to read %i, but was %i.\n", i2s_read_size, bytes_read);
            while(true);
        }
        if(err != ESP_OK) {
            Serial.println("error while reading I2S.");
            while(true);
        }

        i2s_read_done = true;

        check_i2s_speed();

        vTaskDelay(1);

        Serial.print("read took ");
        Serial.print(micros() - time);
        Serial.println("µs");
        time = micros();
    }
}

const int fft_normalization_store_size = 20;
double fft_normalization_max_values[fft_normalization_store_size] = {};
int fft_normalization_max_value_index = 0;

double* get_binned_values(double fft_data[], double *result) {
    double max_value = 0, min_value = __INT_MAX__;
    for(int result_index = 0; result_index < fft_frequency_bin_count; result_index++) {
        int index_min = fft_frequency_bin_lower_indices[result_index];
        int index_max = fft_frequency_bin_upper_indices[result_index];
        
        result[result_index] = 0;   // directly save max value in result[]
        for(int bin_index = index_min; bin_index < index_max; bin_index++) {
            double_t value = fft_data[bin_index] * fft_frequency_bin_multiplier[result_index];
            if(value > result[result_index]) {
                result[result_index] = value;
            }
            if(value > max_value) {
                max_value = value;
            }
            if(value < min_value) {
                min_value = value;
            }
        }
    }

    // store current max value
    fft_normalization_max_values[fft_normalization_max_value_index] = max_value;

    

    // calculate average max value
    double avg_max_value = 0;
    for(int i = 0; i < fft_normalization_max_value_index; i++) {
        avg_max_value += fft_normalization_max_values[i];
    }
    avg_max_value /= fft_normalization_store_size;

    fft_normalization_max_value_index++;
    if(fft_normalization_max_value_index >= fft_normalization_store_size) {
        fft_normalization_max_value_index = 0;
    }

    // debug: print fft data
    /*Serial.print("fft data: ");
    for(int i = 0; i < 10; i++) {
        Serial.print(fft_data[i]);
        Serial.print(", ");
    }
    Serial.println();*/

    // normalize data
    //Serial.println("binned values:");
    max_value -= min_value;
    for(int result_index = 0; result_index < fft_frequency_bin_count; result_index++) {
        result[result_index] = (result[result_index] - min_value) / max_value;
        if(result[result_index] > 1.0) {
            result[result_index] = 1.0;
        }
        // debug: print result
        //Serial.print(result[result_index], 16);
        //Serial.print(", ");
    }
    //Serial.println();

    return result;
}

// analzye takes 15900µs = 15,9ms
// without FFT Hamming-windowing: 12,2ms
// with RTA display: 30ms
void analyze_data(void * pvParameters) {

    // OLED stuff
    if(!oledDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("error initializing SSD1306 display.");
        while(true);
    }
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);         // small font size (height 8px)
    oledDisplay.setTextColor(WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("analyzing...");
    oledDisplay.display();

    unsigned long time_end = 0;

    while(true) {
        // wait until reading is done
        while(!i2s_read_done) {
        }

        // debug: print fft input array
        /*for(int i = 0; i < sample_count; i++) {
            Serial.print(samples[i]);
            Serial.print(", ");
        }
        Serial.println();*/

        // fill fft_imag array and convert input array from int32_t to double_t
        for(int i = 0; i < sample_count; i++) {
            fft_imag[i] = 0;
            fft_real[i] = (double) samples[i];
        }

        i2s_read_done = false;

        FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(FFT_FORWARD);
        FFT.ComplexToMagnitude();

        // debug: print fft output array
        /*Serial.println("FFT:");
        for(int i = 0; i < sample_count; i++) {
            Serial.print(fft_real[i]);
            Serial.print(", ");
        }
        Serial.println();*/

        /*int freq = FFT.MajorPeakParabola();

        char c[15];
        String s = "";
        if(freq < 1000) {
            sprintf(c, "  %i", freq);
        } else if(freq < 10000) {
            sprintf(c, " %i", freq);
        } else {
            sprintf(c, "%i", freq);
        }
        s = s + c;

        oledDisplay.clearDisplay();
        oledDisplay.setCursor(0, 0);
        oledDisplay.printf("f: %s Hz\n", s);
        oledDisplay.print("analyze ");
        oledDisplay.print(micros() - time_end);
        oledDisplay.println("");
        oledDisplay.display();*/

        oledDisplay.clearDisplay();
        oledDisplay.setCursor(0, 0);
        double binned_data[fft_frequency_bin_count] = {};
        get_binned_values(fft_real, binned_data);
        const float rta_max_bar_height = 32.0;
        for(int i = 0; i < fft_frequency_bin_count; i++) {
            int x_offset = i * 8;
            double height_d = std::floor((binned_data[i] * rta_max_bar_height) + 0.5);
            int height = height_d;
            int y_offset = rta_max_bar_height - height;
            for(int x = 0; x < 8; x++) {
                oledDisplay.drawFastVLine(x_offset + x, y_offset, 32, WHITE);
            }
        }
        oledDisplay.display();

        vTaskDelay(1);

        Serial.print("analyze took ");
        Serial.print(micros() - time_end);
        Serial.println("µs");

        time_end = micros();

    }
}








void loop() {
    vTaskDelete(NULL);
}


/*
TODO: 
    - queue hinzufügen -> queue geht nicht, weil FFT nicht unterschiedlich viele Elemente erwarten kann
    --> Ringpuffer hinzufügen, feste Größe z.b. 4
        - i2s_read() pusht gelesene samples in puffer
        - analyze() popt alle verfügbaren (4) samples aus queue und berechnet FFT
            - FFT auf 4 * samples_count anpassen (vorher angucken, ob 4 auf einander folgende sample-arrays aneinander passen)
            - eventuell Semaphore, dass analyze() immer exakt nach 4 neuen sample-Arrays ausgeführt wird
    - Normalisierung des RTA über gespeicherten durchschnitts-max-value
        - max-value nutzen, evtl. Anzahl der gespeicherten max-Values noch erhöhen (20 -> 200 ?)
    - RTA-multiplier anpassen

*/