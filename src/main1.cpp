// #include <mbed.h>                     // Include the mbed framework
// #include "drivers/LCD_DISCO_F429ZI.h" // Include the LCD display library for the ST DISCO-F429ZI board
// #include "arm_math.h"                 // Include the CMSIS-DSP library

// // Definitions for control registers and configurations for an SPI-connected device
// #define CTRL_REG1 0x20
// #define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
// #define CTRL_REG4 0x23
// #define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
// #define SPI_FLAG 1
// #define OUT_X_L 0x28

// // Definitions for scaling, buffer sizes, thresholds, and filter parameters
// #define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
// #define SAMPLING_RATE 100
// #define BUFFER_SIZE 100
// #define NUM_TAPS 51
// #define TREMOR_THRESHOLD 0.7  //Tremor Threshold to remove noise from gyroscope data when its stationary.
// //If getting tremor more than 0.7 when gyroscope is stationary, please configure TREMOR_THRESHOLD by placing the gyroscope stationary on a flat surface and the tremor value in the serial monitor update this value to that, maybe a 0.1 over that value.
// #define NUM_RECENT_EVENTS 10
// #define NO_TREMOR_TIME_THRESHOLD (10 * SAMPLING_RATE) // 10 seconds converted to number of samples at given SAMPLING_RATE

// EventFlags flags;
// SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // SPI bus configuration

// LCD_DISCO_F429ZI lcd; // Create an LCD object

// // State buffer and coefficients for the FIR filter
// static float32_t firStateF32[BUFFER_SIZE + NUM_TAPS - 1];

// // FIR coefficients for a band-pass filter designed externally
// const float32_t firCoeffs32[NUM_TAPS] = {
//     0.00144587f, 0.00212609f, 0.00299537f, 0.00403436f, 0.00508687f,
//     0.00583538f, 0.00582272f, 0.00452492f, 0.00146761f, -0.00363491f,
//     -0.01074745f, -0.0194218f, -0.02877348f, -0.03754508f, -0.04425722f,
//     -0.04742975f, -0.04583883f, -0.03876484f, -0.02618269f, -0.00885355f,
//     0.011708f, 0.03339534f, 0.0537803f, 0.07045205f, 0.08136961f,
//     0.08516859f, 0.08136961f, 0.07045205f, 0.0537803f, 0.03339534f,
//     0.011708f, -0.00885355f, -0.02618269f, -0.03876484f, -0.04583883f,
//     -0.04742975f, -0.04425722f, -0.03754508f, -0.02877348f, -0.0194218f,
//     -0.01074745f, -0.00363491f, 0.00146761f, 0.00452492f, 0.00582272f,
//     0.00583538f, 0.00508687f, 0.00403436f, 0.00299537f, 0.00212609f,
//     0.00144587f};

// // CMSIS-DSP FIR instance
// arm_fir_instance_f32 S;

// std::array<float, BUFFER_SIZE> gyro_buffer = {};
// int buffer_index = 0;

// std::array<std::chrono::milliseconds, NUM_RECENT_EVENTS> event_times;
// size_t event_index = 0;
// bool is_first_event = true;
// int no_tremor_counter = 0;      // Counter to track the duration of no tremor detection
// bool message_displayed = false; // Flag to control the display of the no tremor message

// // Callback for handling SPI events
// void spi_cb(int event)
// {
//     flags.set(SPI_FLAG);
// }

// // Apply FIR filter to input sample
// void apply_fir(float input, float *output)
// {
//     // Process one sample at a time
//     arm_fir_f32(&S, &input, output, 1);
// }

// // Add gyroscope reading to the buffer
// void add_to_buffer(float gx)
// {
//     gyro_buffer[buffer_index++] = gx;
//     if (buffer_index >= BUFFER_SIZE)
//         buffer_index = 0;
// }

// // Compute the average of the buffer for tremor detection
// float compute_average()
// {
//     float sum = 0.0f;
//     for (float val : gyro_buffer)
//     {
//         sum += val;
//     }
//     return sum / BUFFER_SIZE;
// }

// // Detect if the computed average indicates a tremor
// bool detect_tremor(float average)
// {
//     return fabs(average) > TREMOR_THRESHOLD;
// }

// // Calculate the frequency of detected tremors
// float calculate_frequency()
// {
//     if (is_first_event)
//         return 0.0f;

//     float total_interval = 0.0f;
//     size_t valid_intervals = 0;
//     for (size_t i = 0; i < NUM_RECENT_EVENTS - 1; i++)
//     {
//         if (event_times[i + 1].count() == 0)
//             break;
//         auto interval = (event_times[i + 1] - event_times[i]).count();
//         if (interval > 0)
//         {
//             total_interval += interval;
//             valid_intervals++;
//         }
//     }
//     if (valid_intervals == 0)
//         return 0.0f;
//     return 1000.0f / (total_interval / valid_intervals);
// }

// // Log the time of a tremor event
// void log_event_time()
// {
//     event_times[event_index++] = std::chrono::duration_cast<std::chrono::milliseconds>(
//         Kernel::Clock::now().time_since_epoch());
//     if (event_index >= NUM_RECENT_EVENTS)
//         event_index = 0;
//     if (is_first_event)
//         is_first_event = false;
// }

// // Setup the LCD display at startup
// void setup_lcd()
// {
//     lcd.Clear(LCD_COLOR_BLACK);
//     lcd.SetBackColor(LCD_COLOR_BLACK);
//     lcd.SetTextColor(LCD_COLOR_WHITE);
//     lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
//     lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
// }

// // Display tremor intensity on the LCD
// void display_intensity(const char *intensity)
// {
//     char label[64] = "Intensity: ";
//     lcd.SetTextColor(LCD_COLOR_WHITE);
//     lcd.DisplayStringAt(0, LINE(9), (uint8_t *)label, CENTER_MODE);

//     // Determine the position for the intensity value based on the length of "Intensity: "
//     int offset = strlen(label) * 15; // Position calculation for the intensity value

//     //Setting different LCD Color for different intensities
//     if (strstr(intensity, "Low"))
//     {
//         lcd.SetTextColor(LCD_COLOR_GREEN);
//         lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Low", LEFT_MODE);
//     }
//     else if (strstr(intensity, "Medium"))
//     {
//         lcd.SetTextColor(LCD_COLOR_BLUE);
//         lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Medium", LEFT_MODE);
//     }
//     else if (strstr(intensity, "High"))
//     {
//         lcd.SetTextColor(LCD_COLOR_RED);
//         lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"High", LEFT_MODE);
//     }
// }

// // Clear a line on the LCD
// void clear_line(uint8_t line)
// {
//     char empty_line[64]; // Adjusting the length based on the LCD width
//     memset(empty_line, ' ', sizeof(empty_line) - 1);
//     empty_line[sizeof(empty_line) - 1] = '\0';
//     lcd.DisplayStringAt(0, line, (uint8_t *)empty_line, CENTER_MODE);
// }

// int main()
// {
//     setup_lcd();              // Initial setup of the LCD
//     spi.format(8, 3);         // Setup SPI format
//     spi.frequency(1'000'000); // SPI frequency

//     // Initialize and configure the FIR filter
//     arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32, firStateF32, 1);

//     // Configure control registers via SPI
//     uint8_t write_buf[32], read_buf[32];
//     write_buf[0] = CTRL_REG1;
//     write_buf[1] = CTRL_REG1_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     write_buf[0] = CTRL_REG4;
//     write_buf[1] = CTRL_REG4_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     while (true)
//     {
//         // Main loop for reading gyro data, applying the FIR filter, and checking for tremors
//         uint16_t raw_gx;
//         float gx, filtered_gx;
//         write_buf[0] = OUT_X_L | 0x80 | 0x40; // Setup for reading
//         spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
//         flags.wait_all(SPI_FLAG);

//         // Convert the data
//         raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
//         gx = ((float)raw_gx) * SCALING_FACTOR;

//         // Apply the FIR filter
//         apply_fir(gx, &filtered_gx);

//         // Add data to buffer and analyze
//         add_to_buffer(filtered_gx);
//         float average = compute_average();

//         if (detect_tremor(average))
//         {
//             no_tremor_counter = 0; // Reset counter on tremor detection
//             message_displayed = false;
//             log_event_time();
//             float frequency = calculate_frequency();
//             // COde to limit frequency to 3-6Hz, handle detected tremor, including displaying on the LCD
//             if (frequency >= 3.0f && frequency <= 6.0f)
//             {
//                 char buf[64];
//                 char intensity[64];
//                 printf("Tremor detected: %4.5f degrees/s, Frequency: %3.2f Hz\n", average, frequency); // For serial Monitor check
//                 snprintf(buf, sizeof(buf), "Tremor Freq: %3.2f Hz", frequency);
//                 lcd.Clear(LCD_COLOR_BLACK); // Resetting the LCD screen to black
//                 // Setting Intensity based on tremor frequeny
//                 if (frequency >= 3.0f && frequency <= 4.0f)
//                 {
//                     snprintf(intensity, sizeof(buf), "Intensity: Low");
//                 }
//                 else if (frequency >= 4.0f && frequency <= 5.0f)
//                 {
//                     snprintf(intensity, sizeof(buf), "Intensity: Medium");
//                 }
//                 else
//                 {
//                     snprintf(intensity, sizeof(buf), "Intensity: High");
//                 }
//                 lcd.SetTextColor(LCD_COLOR_WHITE);
//                 clear_line(LINE(7));
//                 clear_line(LINE(8));
//                 lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
//                 lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
//                 lcd.DisplayStringAt(0, LINE(7), (uint8_t *)buf, CENTER_MODE);
//                 display_intensity(intensity);
//             }
//         }
//         else
//         {
//             // Code to handle absence of detected tremor
//             if (!message_displayed)
//             {
//                 no_tremor_counter++;
//                 if (no_tremor_counter >= NO_TREMOR_TIME_THRESHOLD)
//                 {
//                     // Display "No Resting Tremor Detected" on the LCD
//                     lcd.SetTextColor(LCD_COLOR_WHITE);
//                     lcd.Clear(LCD_COLOR_BLACK);
//                     lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
//                     lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
//                     lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"No Resting Tremor", CENTER_MODE);
//                     lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Detected", CENTER_MODE);
//                     message_displayed = true; // Set flag to prevent re-display
//                 }
//             }
//         }

//         thread_sleep_for(1000 / SAMPLING_RATE); // Sleep to maintain the sampling rate
//     }
// }