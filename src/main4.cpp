// #include <mbed.h>
// #include <cmath> // For math functions

// #define CTRL_REG1 0x20
// #define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
// #define CTRL_REG4 0x23
// #define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
// #define SPI_FLAG 1
// #define OUT_X_L 0x28

// EventFlags flags;

// void spi_cb(int event) {
//     flags.set(SPI_FLAG);
// }

// #define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// // Parameters for tremor detection
// #define SAMPLING_RATE 100 // Sampling rate in Hz
// #define BUFFER_SIZE 100 // Number of samples to keep in buffer
// #define TREMOR_THRESHOLD 0.7 // Threshold for tremor detection in degrees/s

// FIR filter parameters
// constexpr size_t NUM_TAPS = 51;
// std::array<float, NUM_TAPS> fir_coeff = {
//     0.00144587f,  0.00212609f,  0.00299537f,  0.00403436f,  0.00508687f,
//     0.00583538f,  0.00582272f,  0.00452492f,  0.00146761f, -0.00363491f,
//     -0.01074745f, -0.0194218f,  -0.02877348f, -0.03754508f, -0.04425722f,
//     -0.04742975f, -0.04583883f, -0.03876484f, -0.02618269f, -0.00885355f,
//     0.011708f,    0.03339534f,  0.0537803f,   0.07045205f,  0.08136961f,
//     0.08516859f,  0.08136961f,  0.07045205f,  0.0537803f,   0.03339534f,
//     0.011708f,   -0.00885355f, -0.02618269f, -0.03876484f, -0.04583883f,
//     -0.04742975f, -0.04425722f, -0.03754508f, -0.02877348f, -0.0194218f,
//     -0.01074745f, -0.00363491f,  0.00146761f,  0.00452492f,  0.00582272f,
//     0.00583538f,  0.00508687f,  0.00403436f,  0.00299537f,  0.00212609f,
//     0.00144587f
// };
// std::array<float, NUM_TAPS> buffer = {}; // Initialize buffer to zero

// // Circular buffer for storing gyro data
// float gyro_buffer[BUFFER_SIZE] = {0};
// int buffer_index = 0;

// // Function to add data to buffer
// void add_to_buffer(float gx) {
//     gyro_buffer[buffer_index++] = gx;
//     if (buffer_index >= BUFFER_SIZE) buffer_index = 0;
// }

// // Function to compute the average of the buffer
// float compute_average() {
//     float sum = 0.0;
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         sum += gyro_buffer[i];
//     }
//     return sum / BUFFER_SIZE;
// }

// // Function to detect tremor
// bool detect_tremor(float average) {
//     return fabs(average) > TREMOR_THRESHOLD;
// }

// // Function to apply the FIR filter
// float apply_fir(float new_sample) {
//     // Move old data
//     for (int i = NUM_TAPS - 1; i > 0; i--) {
//         buffer[i] = buffer[i - 1];
//     }
//     // Insert new sample at the beginning
//     buffer[0] = new_sample;

//     // Apply FIR filter (dot product of buffer and coefficients)
//     float output = 0.0f;
//     for (size_t i = 0; i < NUM_TAPS; i++) {
//         output += buffer[i] * fir_coeff[i];
//     }
//     return output;
// }

// int main() {
//     // Initialize the SPI object with specific pins.
//     SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
//     spi.format(8, 3);
//     spi.frequency(1'000'000);

//     // Configure control registers
//     uint8_t write_buf[32], read_buf[32];
//     write_buf[0] = CTRL_REG1; write_buf[1] = CTRL_REG1_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     write_buf[0] = CTRL_REG4; write_buf[1] = CTRL_REG4_CONFIG;
//     spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
//     flags.wait_all(SPI_FLAG);

//     while (1) {
//         // Read gyro data
//         uint16_t raw_gx;
//         float gx;
//         write_buf[0] = OUT_X_L | 0x80 | 0x40; // Setup for reading
//         spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
//         flags.wait_all(SPI_FLAG);

//         // Convert the data
//         raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
//         gx = ((float) raw_gx) * SCALING_FACTOR;

//         // Filter the gyro data
//         float filtered_gx = apply_fir(gx);

//         // Add data to buffer and analyze
//         add_to_buffer(filtered_gx);
//         float average = compute_average();
//         if (detect_tremor(average)) {
//             printf("Tremor detected: %4.5f degrees/s\n", average);
//         }

//         thread_sleep_for(1000 / SAMPLING_RATE); // Maintain sampling rate
//     }
// }
