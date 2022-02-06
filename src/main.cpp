#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// I2S stuff
QueueHandle_t i2s_event_queue;              // i2s event queue
const int i2s_event_queue_length = 16;      // max amount of events in i2s event queue
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int sample_count = 512;               // sample count of each i2s reading (512)
const int i2s_sampling_rate = 44100;        // Hz (44100)
const int i2s_buffer_size = 512;            // size of i2s DMA buffers (128)
const int i2s_buffer_count = 4;             // count of i2s DMA buffers (4)
const int i2s_read_size = 4 * sample_count; // 4 bytes per sample * #samples
int32_t samples0[sample_count] = {0};       // destinations of each i2s reading
int32_t samples1[sample_count] = {0};       // destinations of each i2s reading

// FFT stuff
double fft_imag[sample_count] = {};         // FFT imaginary input values (all 0)
double fft_real[sample_count] = {};         // FFT real input values are copied from samples[] before each FFT analysis
arduinoFFT FFT = arduinoFFT(fft_real, fft_imag, sample_count, i2s_sampling_rate);
const int fft_frequency_bin_count = 16;     // indices:                    0     1     2     3     4     5     6      7      8      9     10     11     12     13      14      15
const int16_t fft_frequency_bins[fft_frequency_bin_count]              = {100,  140,  196,  274,  384,  538,  753,  1054,  1476,  2066,  2893,  4050,  5669,  7937,  11112,  15557}; // Hz
const int16_t fft_frequency_bin_lower_indices[fft_frequency_bin_count] = {0,    1,    2,    3,    4,    5,    8,    11,    15,    21,    29,    40,    56,    79,    111,    155};
const int16_t fft_frequency_bin_upper_indices[fft_frequency_bin_count] = {  1,    2,    3,    4,    5,    8,   11,    15,    21,    29,    40,    56,    79,   111,    155,    256};
const float_t fft_frequency_bin_multiplier[fft_frequency_bin_count] =   {0.0001,0.00025,0.1,0.3,  0.5,  0.6,  0.7,   0.8,   0.9,   1.3,   1.4,   1.5,   1.5,   1.5,    1.5,    1.5};

// RTA stuff
const int rta_max_bar_height = 32;          // bar height of each RTA bar in px
const int rta_normalization_store_size = 100;// count of last max values to be stored, only applicable for rta_gain_adaptation = RTA_ADAPTIVE_AVG_GAIN
double rta_normalization_max_values[rta_normalization_store_size] = {};
int rta_normalization_max_value_index = 0;
#define RTA_CONSTANT_GAIN 0x00              // use constant max value for constant RTA gain
#define RTA_ADAPTIVE_GAIN 0x01              // use current max value for adaptive RTA gain with fastest response
#define RTA_ADAPTIVE_AVG_GAIN 0x02          // use average max value for adaptive RTA gain with slower response -> better readable RTA
const int rta_gain_adaptation = RTA_ADAPTIVE_AVG_GAIN;
const double rta_constant_gain_value = 350000000.0; // RTA gain, only applicable for rta_gain_adaptation = RTA_CONSTANT_GAIN, e.g. 350000000.0

// File output stuff
bool csv_file_output_active = true;         // enable/disable CSV file output (when true, output is enabled/disabled automatically depending on presence of SD card)
File csv_file;                              // CSV file for outputting
const String csv_file_name = "rta.csv";     // CSV file to use

// parallelization stuff
void read_i2s(void * pvParameters);         // i2s read method
void analyze_data(void * pvParameters);     // FFT analyze method
int32_t samples_write_lock = 0;             // parallelization semaphore lock
int32_t samples_read_lock = -1;             // parallelization read lock

// OLED Display stuff
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
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("initializing...");
    oledDisplay.display();

    // CSV stuff
    if(csv_file_output_active) {
        SPI.begin(14, 2, 15);
        if(!SD.begin(13)){
            Serial.println("Failed to mount card");
            csv_file_output_active = false;         // deactivate feature 
        } else {
            csv_file = SD.open("/" + csv_file_name, FILE_APPEND);
            if(csv_file) {
                // write headers
                csv_file.print("\n\n\n");
                for(int i = 0; i < fft_frequency_bin_count; i++) {
                    csv_file.printf("%i;", fft_frequency_bins[i]);
                }
                csv_file.print("\n");
                csv_file.close();
                
                oledDisplay.setCursor(0, 8);
                oledDisplay.println("appending to " + csv_file_name);
                oledDisplay.display();
            } else {
                Serial.println("Failed to open CSV file");
                csv_file_output_active = false;         // deactivate feature 
            }
        }
    }
    if(!csv_file_output_active) {
        oledDisplay.setCursor(0, 8);
        oledDisplay.println("no file output.");
        oledDisplay.display();
    }
    delay(3000);

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
        .bck_io_num = 23,   // Serial Clock (BCK on Mic)    green cable,  works: 14)
        .ws_io_num = 25,    // Word Select (LRCL on Mic)    blue cable,   works: 13)
        .data_out_num = I2S_PIN_NO_CHANGE, // not used (only for speakers)
        .data_in_num = 12   // Serial Data (DOUT on Mic)    purple cable, works: 12)
    };

    err = i2s_driver_install(I2S_PORT, &i2s_config, i2s_event_queue_length, &i2s_event_queue);
    if (err != ESP_OK) {
        Serial.printf("Failed installing I2S driver: %d\n", err);
        while (true);
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting I2S pin: %d\n", err);
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

    Serial.println("setup done");
}

// idea from https://github.com/esikora/M5StickC_AudioVisLed/blob/main/src/M5StickC_AudiVisLedApp.cpp
void check_i2s_speed() {
    i2s_event_t i2s_event;
    uint16_t rx_done_event_count = 0;
    
    for(int i = 0; i < uxQueueMessagesWaiting(i2s_event_queue); i++) {
        if(xQueueReceive(i2s_event_queue, (void*) &i2s_event, 0) == pdTRUE) {
            if(i2s_event.type == I2S_EVENT_RX_DONE) {
                rx_done_event_count++;
            }
        }
    }

    if(rx_done_event_count > (sample_count / i2s_buffer_size)) {
        Serial.printf("too many I2S RX DONE events in queue: %d\n", rx_done_event_count);
        if(rx_done_event_count == i2s_event_queue_length) {
            // only stop program if there is no chance of catching up
            while(true);
        }
    }
    if(rx_done_event_count < (sample_count / i2s_buffer_size)) {
        Serial.printf("not enough I2S RX DONE events in queue: %d\n", rx_done_event_count);
        while(true);
    }
}

// read takes 11000µs = 11ms
void read_i2s(void * pvParameters) {
    size_t bytes_read;
    esp_err_t err;

    //unsigned int time = 0;

    while(true) {
        switch(samples_write_lock) {
            case 0:
                // write lock was on samples0 last time --> set to samples1 and start reading
                while(samples_read_lock == 1) {
                    // wait, does basically never happen
                }        
                samples_write_lock = 1;
                err = i2s_read(I2S_PORT, &samples1, i2s_read_size, &bytes_read, portMAX_DELAY);     // this alone takes 10,578 - 11,567ms   (avg 11,078ms, over 380 iterations)
                break;
            case 1:
                // write lock was on samples1 last time --> set to samples0 and start reading
                while(samples_read_lock == 0) {
                    // wait, does basically never happen
                }
                samples_write_lock = 0;
                err = i2s_read(I2S_PORT, &samples0, i2s_read_size, &bytes_read, portMAX_DELAY);      // this alone takes 10,578 - 11,567ms   (avg 11,078ms, over 380 iterations)
                break;
        }
        
        if(bytes_read != i2s_read_size) {
            Serial.printf("read incorrect amount of samples: Expected to read %i, but was %i.\n", i2s_read_size, bytes_read);
            while(true);
        }
        if(err != ESP_OK) {
            Serial.println("error while reading I2S.");
            while(true);
        }

        check_i2s_speed();

        vTaskDelay(1);

        /*Serial.print("read took ");
        Serial.print(micros() - time);
        Serial.println("µs");
        time = micros();*/
    }
}


double* calc_rta_bins(double fft_data[], double *result) {
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

    double avg_max_value = 0;
    if(rta_gain_adaptation == RTA_ADAPTIVE_AVG_GAIN) {
        // store current max value
        rta_normalization_max_values[rta_normalization_max_value_index] = max_value;

        // calculate average max value
        for(int i = 0; i < rta_normalization_store_size; i++) {
            int index = i + rta_normalization_max_value_index;
            if(index > rta_normalization_store_size) {
                index -= rta_normalization_store_size;
            }
            avg_max_value += rta_normalization_max_values[index];
        }
        avg_max_value /= rta_normalization_store_size;

        rta_normalization_max_value_index++;
        if(rta_normalization_max_value_index >= rta_normalization_store_size) {
            rta_normalization_max_value_index = 0;
        }
    }

    // normalize data
    max_value -= min_value;
    for(int result_index = 0; result_index < fft_frequency_bin_count; result_index++) {
        switch (rta_gain_adaptation) {
            case RTA_CONSTANT_GAIN:
                result[result_index] = (result[result_index] - min_value) / rta_constant_gain_value;
                break;
            case RTA_ADAPTIVE_GAIN:
                result[result_index] = (result[result_index] - min_value) / max_value;
                break;
            default:
            case RTA_ADAPTIVE_AVG_GAIN:
                result[result_index] = (result[result_index] - min_value) / avg_max_value;
                break;
        }
        if(result[result_index] > 1.0) {
            result[result_index] = 1.0;
        }
    }

    return result;
}

// analzye takes 15900µs = 15,9ms
// without FFT Hamming-windowing: 12,2ms
// with RTA display: 30ms
// with RTA, without FFT Hamming-windowing: 26,98ms
void analyze_data(void * pvParameters) {
    Serial.println("analyzing...");

    // OLED stuff, have to do this again in order for the display to print the correct result
    if(!oledDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("error initializing SSD1306 display.");
        while(true);
    }
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println("analyzing...");
    oledDisplay.display();
    
    if(csv_file_output_active) {
        csv_file = SD.open("/" + csv_file_name, FILE_APPEND);
        if(csv_file) {
            Serial.println("opened " + csv_file_name + " for analyzing");
        } else {
            Serial.println("Failed to open CSV file " + csv_file_name);
            csv_file_output_active = false;         // deactivate feature 
        }
    }

    unsigned int loop_counter = 0;
    //unsigned int time_end = 0;

    while(true) {
        // fill fft_imag array and convert input array from int32_t to double_t
        // this loop takes 122µs = 0,122ms = 0,00122 s
        switch(samples_write_lock) {
            case 0:
                // write lock is on samples0 --> read samples1
                samples_read_lock = 1;
                for(int i = 0; i < sample_count; i++) {
                    fft_imag[i] = 0;
                    fft_real[i] = (double) samples1[i];
                }
                break;
            case 1:
                // write lock is on samples1 --> read samples0
                samples_read_lock = 0;
                for(int i = 0; i < sample_count; i++) {
                    fft_imag[i] = 0;
                    fft_real[i] = (double) samples0[i];
                }
                break;
        }
        samples_read_lock = -1;

        FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(FFT_FORWARD);
        FFT.ComplexToMagnitude();

        oledDisplay.clearDisplay();
        oledDisplay.setCursor(0, 0);

        double binned_data[fft_frequency_bin_count] = {};
        calc_rta_bins(fft_real, binned_data);
        for(int i = 0; i < fft_frequency_bin_count; i++) {
            int x_offset = i * 8;
            int height = std::floor((binned_data[i] * rta_max_bar_height) + 0.5);
            int y_offset = rta_max_bar_height - height;
            // oledDisplay.drawRect(x_offset, y_offset, 8, height, WHITE);      with this line, drawing takes approx. 13,35ms (non-filled rectangles)
            for(int x = 0; x < 8; x++) {  //    with this loop, drawing takes approx. 13,37ms (filled rectangles)
                oledDisplay.drawFastVLine(x_offset + x, y_offset, 32, WHITE);
            }
            if(csv_file_output_active) {
                csv_file.printf("%f;", binned_data[i]);
            }
        }
        oledDisplay.display();

        if(csv_file_output_active) {
            csv_file.print("\n");
            
            loop_counter++;
            // flush csv file every 100 iterations (takes approx. 20ms)
            if(loop_counter == 100) {
                csv_file.flush();
                loop_counter = 0; // prevent overflow
            }
        }

        vTaskDelay(1);

        /*Serial.print("analyze took ");
        Serial.print(micros() - time_end);
        Serial.println("µs");

        time_end = micros();*/
    }
}



void loop() {
    vTaskDelete(NULL);
}

