// #include <mbed.h>
// #include "drivers/LCD_DISCO_F429ZI.h"
// #include "arm_math.h"
// #include <algorithm>
// #include <array>
// #include <cstring>

// // Definitions and Constants
// #define CTRL_REG1 0x20
// #define CTRL_REG1_CONFIG 0b01101111
// #define CTRL_REG4 0x23
// #define CTRL_REG4_CONFIG 0b00010000
// #define SPI_FLAG 1
// #define OUT_X_L 0x28
// #define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
// #define SAMPLING_RATE 100
// #define BUFFER_SIZE 256  // FFT requires a power of 2
// #define NUM_RECENT_EVENTS 10
// #define TREMOR_THRESHOLD 0.50
// #define FREQUENCY_BUFFER_SIZE 3
// #define NO_TREMOR_TIME_THRESHOLD (8 * SAMPLING_RATE)

// EventFlags flags;
// SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
// LCD_DISCO_F429ZI lcd;

// // FFT Variables
// arm_rfft_fast_instance_f32 fft_instance;
// float input_buffer[BUFFER_SIZE];
// float output_buffer[BUFFER_SIZE];
// float magnitudes[BUFFER_SIZE / 2];

// std::array<float, BUFFER_SIZE> gyro_buffer_x, gyro_buffer_y, gyro_buffer_z;
// int buffer_index = 0;

// std::array<std::chrono::milliseconds, NUM_RECENT_EVENTS> event_times;
// size_t event_index = 0;
// bool is_first_event = true;
// int no_tremor_counter = 0;
// bool message_displayed = false;
// std::array<float, FREQUENCY_BUFFER_SIZE> frequency_buffer = {};
// int frequency_index = 0;
// bool frequency_buffer_full = false;

// void spi_cb(int event) {
//     flags.set(SPI_FLAG);
// }

// void add_to_buffer(float val, std::array<float, BUFFER_SIZE>& buffer, int& index) {
//     buffer[index++] = val;
//     if (index >= BUFFER_SIZE) index = 0;
// }

// float compute_average(const std::array<float, BUFFER_SIZE>& buffer) {
//     float sum = 0.0f;
//     for (auto val : buffer) sum += val;
//     return sum / BUFFER_SIZE;
// }

// bool detect_tremor(float magnitude) {
//     return fabs(magnitude) > TREMOR_THRESHOLD;
// }

// void reset_tremor_event_tracking() {
//     is_first_event = true;
//     event_index = 0;
//     std::fill(event_times.begin(), event_times.end(), std::chrono::milliseconds(0));
// }

// void reset_frequency_buffer() {
//     for (int i = 0; i < FREQUENCY_BUFFER_SIZE; i++) {
//         frequency_buffer[i] = 0.0f;
//     }
//     frequency_index = 0;
//     frequency_buffer_full = false;
// }

// void log_event_time() {
//     event_times[event_index++] = std::chrono::duration_cast<std::chrono::milliseconds>(
//         Kernel::Clock::now().time_since_epoch());
//     if (event_index >= NUM_RECENT_EVENTS) event_index = 0;
//     if (is_first_event) is_first_event = false;
// }

// float calculate_frequency() {
//     if (is_first_event || event_index < 2) return 0.0f;
//     float total_interval = 0.0f;
//     size_t valid_intervals = 0;
//     for (size_t i = 1; i < event_index; i++) {
//         auto interval = (event_times[i] - event_times[i - 1]).count();
//         if (interval > 0) {
//             total_interval += interval;
//             valid_intervals++;
//         }
//     }
//     if (valid_intervals == 0) return 0.0f;
//     float average_interval = total_interval / valid_intervals;
//     return 1000.0f / average_interval;
// }

// float calculate_average_frequency() {
//     float total = 0.0f;
//     for (int i = 0; i < FREQUENCY_BUFFER_SIZE; i++) {
//         total += frequency_buffer[i];
//     }
//     return total / FREQUENCY_BUFFER_SIZE;
// }

// void setup_lcd() {
//     lcd.Clear(LCD_COLOR_BLACK);
//     lcd.SetBackColor(LCD_COLOR_BLACK);
//     lcd.SetTextColor(LCD_COLOR_WHITE);
//     lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
//     lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
// }

// void display_intensity(const char *intensity) {
//     char label[64] = "Intensity: ";
//     lcd.SetTextColor(LCD_COLOR_WHITE);
//     lcd.DisplayStringAt(0, LINE(9), (uint8_t *)label, LEFT_MODE);
//     int offset = strlen(label) * 100;
//     if (strstr(intensity, "Low")) {
//         lcd.SetTextColor(LCD_COLOR_GREEN);
//         lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Low", LEFT_MODE);
//     } else if (strstr(intensity, "Medium")) {
//         lcd.SetTextColor(LCD_COLOR_BLUE);
//         lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Medium", LEFT_MODE);
//     } else if (strstr(intensity, "High")) {
//         lcd.SetTextColor(LCD_COLOR_RED);
//         lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"High", LEFT_MODE);
//     }
// }

// void clear_line(uint8_t line) {
//     char empty_line[64];
//     memset(empty_line, ' ', sizeof(empty_line) - 1);
//     empty_line[sizeof(empty_line) - 1] = '\0';
//     lcd.DisplayStringAt(0, line, (uint8_t *)empty_line, CENTER_MODE);
// }

// void no_tremor_detected() {
//     if (!message_displayed) {
//         no_tremor_counter++;
//         if (no_tremor_counter >= NO_TREMOR_TIME_THRESHOLD) {
//             lcd.SetTextColor(LCD_COLOR_WHITE);
//             lcd.Clear(LCD_COLOR_BLACK);
//             lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
//             lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
//             lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"No Resting Tremor", CENTER_MODE);
//             lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Detected", CENTER_MODE);
//             message_displayed = true;
//         }
//     }
// }

// void calculate_fft_and_magnitude(float* input, float* output, float* magnitudes, size_t length) {
//     arm_rfft_fast_f32(&fft_instance, input, output, 0);
//     for (size_t i = 0; i < length / 2; i++) {
//         magnitudes[i] = sqrtf(output[2 * i] * output[2 * i] + output[2 * i + 1] * output[2 * i + 1]);
//     }
// }

// float find_peak_frequency(float* magnitudes, size_t length, float sampling_rate) {
//     size_t peak_index = std::distance(magnitudes, std::max_element(magnitudes, magnitudes + length / 2));
//     return (float)peak_index * sampling_rate / length;
// }

// int main() {
//     setup_lcd();
//     spi.format(8, 3);
//     spi.frequency(1'000'000);

//     arm_rfft_fast_init_f32(&fft_instance, BUFFER_SIZE);

//     uint8_t write_buf[32], read_buf[32];
//     write_buf[0] = CTRL_REG1;
//     write_buf[1] = CTRL_REG1_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     write_buf[0] = CTRL_REG4;
//     write_buf[1] = CTRL_REG4_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     while (true) {
//         uint16_t raw_gx, raw_gy, raw_gz;
//         float gx, gy, gz;
//         uint8_t write_buf[7] = {OUT_X_L | 0x80 | 0x40}, read_buf[7];
//         spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
//         flags.wait_all(SPI_FLAG);

//         raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
//         raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
//         raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

//         gx = ((float)raw_gx) * SCALING_FACTOR;
//         gy = ((float)raw_gy) * SCALING_FACTOR;
//         gz = ((float)raw_gz) * SCALING_FACTOR;

//         add_to_buffer(gx, gyro_buffer_x, buffer_index);
//         add_to_buffer(gy, gyro_buffer_y, buffer_index);
//         add_to_buffer(gz, gyro_buffer_z, buffer_index);
//         printf("Buffer Index: %3.2f Hz\n", buffer_index);
//         if (buffer_index == 0) {
//             calculate_fft_and_magnitude(gyro_buffer_x.data(), output_buffer, magnitudes, BUFFER_SIZE);
//             float peak_frequency = find_peak_frequency(magnitudes, BUFFER_SIZE, SAMPLING_RATE);

//             printf("Peak Frequency: %3.2f Hz\n", peak_frequency);

//             if (peak_frequency >= 3.0f && peak_frequency <= 6.0f) {
//                 char buf[64];
//                 char intensity[64];
//                 snprintf(buf, sizeof(buf), "Tremor Freq: %3.2f Hz", peak_frequency);
//                 lcd.Clear(LCD_COLOR_BLACK);

//                 if (peak_frequency >= 3.0f && peak_frequency < 4.0f) {
//                     snprintf(intensity, sizeof(buf), "Intensity: Low");
//                 } else if (peak_frequency >= 4.0f && peak_frequency < 5.0f) {
//                     snprintf(intensity, sizeof(buf), "Intensity: Medium");
//                 } else {
//                     snprintf(intensity, sizeof(buf), "Intensity: High");
//                 }
//                 lcd.SetTextColor(LCD_COLOR_WHITE);
//                 clear_line(LINE(7));
//                 clear_line(LINE(8));
//                 lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
//                 lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
//                 lcd.DisplayStringAt(0, LINE(7), (uint8_t *)buf, CENTER_MODE);
//                 display_intensity(intensity);
//             } else {
//                 no_tremor_detected();
//             }
//         }
//         thread_sleep_for(1000 / SAMPLING_RATE);
//     }
// }
