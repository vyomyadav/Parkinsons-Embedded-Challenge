#include <mbed.h>                     // Include the mbed framework
#include "drivers/LCD_DISCO_F429ZI.h" // Include the LCD display library for the ST DISCO-F429ZI board
#include "arm_math.h"                 // Include the CMSIS-DSP library
#include <algorithm>                  // For std::sort

// Definitions for control registers and configurations for an SPI-connected device
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

// Definitions for scaling, buffer sizes, thresholds, and filter parameters
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define SAMPLING_RATE 100
#define BUFFER_SIZE 100
#define NUM_TAPS 51
#define TREMOR_THRESHOLD 0.50 // Tremor Threshold to remove noise from gyroscope data when its stationary.
// If getting tremor more than 0.7 when gyroscope is stationary, please configure TREMOR_THRESHOLD by placing the gyroscope stationary on a flat surface and the tremor value in the serial monitor update this value to that, maybe a 0.1 over that value.
#define NUM_RECENT_EVENTS 10
#define NO_TREMOR_TIME_THRESHOLD (8 * SAMPLING_RATE) // 10 seconds converted to number of samples at given SAMPLING_RATE
#define FREQUENCY_BUFFER_SIZE 8                 // Defining a frequency buffer to hold frequencies of tremors

EventFlags flags;
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // SPI bus configuration

LCD_DISCO_F429ZI lcd; // Create an LCD object

// State buffer and coefficients for the FIR filter
static float32_t firStateF32[3][BUFFER_SIZE + NUM_TAPS - 1];

// FIR coefficients for a band-pass filter designed externally
const float32_t firCoeffs32[NUM_TAPS] = {
    0.00144587f, 0.00212609f, 0.00299537f, 0.00403436f, 0.00508687f,
    0.00583538f, 0.00582272f, 0.00452492f, 0.00146761f, -0.00363491f,
    -0.01074745f, -0.0194218f, -0.02877348f, -0.03754508f, -0.04425722f,
    -0.04742975f, -0.04583883f, -0.03876484f, -0.02618269f, -0.00885355f,
    0.011708f, 0.03339534f, 0.0537803f, 0.07045205f, 0.08136961f,
    0.08516859f, 0.08136961f, 0.07045205f, 0.0537803f, 0.03339534f,
    0.011708f, -0.00885355f, -0.02618269f, -0.03876484f, -0.04583883f,
    -0.04742975f, -0.04425722f, -0.03754508f, -0.02877348f, -0.0194218f,
    -0.01074745f, -0.00363491f, 0.00146761f, 0.00452492f, 0.00582272f,
    0.00583538f, 0.00508687f, 0.00403436f, 0.00299537f, 0.00212609f,
    0.00144587f
};

// CMSIS-DSP FIR instance
arm_fir_instance_f32 S[3];

std::array<float, BUFFER_SIZE> gyro_buffer_x, gyro_buffer_y, gyro_buffer_z;
int buffer_index = 0;

std::array<std::chrono::milliseconds, NUM_RECENT_EVENTS> event_times;
size_t event_index = 0;
bool is_first_event = true;
int no_tremor_counter = 0;      // Counter to track the duration of no tremor detection
bool message_displayed = false; // Flag to control the display of the no tremor message
std::array<float, FREQUENCY_BUFFER_SIZE> frequency_buffer = {};
int frequency_index = 0;
bool frequency_buffer_full = false; // Flag to check if the buffer is full
int buffer_index_x = 0;
int buffer_index_y = 0;
int buffer_index_z = 0;

// Callback for handling SPI events
void spi_cb(int event)
{
  flags.set(SPI_FLAG);
}

// Definitions for add_to_buffer and compute_average modified to handle different axis
void add_to_buffer(float filtered_val, std::array<float, BUFFER_SIZE>& buffer, int& index) {
    buffer[index++] = filtered_val;
    if (index >= BUFFER_SIZE) index = 0;
}

float compute_average(const std::array<float, BUFFER_SIZE>& buffer) {
    float sum = 0.0f;
    for (auto val : buffer) sum += val;
    return sum / BUFFER_SIZE;
}

// Detect if the computed average indicates a tremor
bool detect_tremor(float magnitude)
{
  return fabs(magnitude) > TREMOR_THRESHOLD;
}

float calculate_frequency() {
  if (is_first_event || event_index < 2)  // Require at least two events to compute a frequency
    return 0.0f;

  float total_interval = 0.0f;
  size_t valid_intervals = 0;
  for (size_t i = 1; i < event_index; i++) {
    auto interval = (event_times[i] - event_times[i - 1]).count();
    if (interval > 0) {
      total_interval += interval;
      valid_intervals++;
    }
  }
  if (valid_intervals == 0)
    return 0.0f;
  float average_interval = total_interval / valid_intervals;
  return 1000.0f / average_interval;  // Convert average interval in milliseconds to frequency in Hz
}

float calculate_average_frequency()
{
  float total = 0.0f;
  for (int i = 0; i < FREQUENCY_BUFFER_SIZE; i++)
  {
    total += frequency_buffer[i];
  }
  return total / FREQUENCY_BUFFER_SIZE;
}



void reset_tremor_event_tracking() {
    is_first_event = true;
    event_index = 0;
    std::fill(event_times.begin(), event_times.end(), std::chrono::milliseconds(0)); // Set all event times to zero
}

void reset_frequency_buffer()
{
  for (int i = 0; i < FREQUENCY_BUFFER_SIZE; i++)
  {
    frequency_buffer[i] = 0.0f; // Reset each element to zero
  }
  frequency_index = 0;           // Reset the index
  frequency_buffer_full = false; // Reset the buffer full flag
}

// Log the time of a tremor event
void log_event_time()
{
  event_times[event_index++] = std::chrono::duration_cast<std::chrono::milliseconds>(
      Kernel::Clock::now().time_since_epoch());
  if (event_index >= NUM_RECENT_EVENTS)
    event_index = 0;
  if (is_first_event)
    is_first_event = false;
}

// Setup the LCD display at startup
void setup_lcd()
{
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_WHITE);
  lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
}

// Display tremor intensity on the LCD
void display_intensity(const char *intensity)
{
  char label[64] = "Intensity: ";
  lcd.SetTextColor(LCD_COLOR_WHITE);
  lcd.DisplayStringAt(0, LINE(9), (uint8_t *)label, LEFT_MODE);

  // Determine the position for the intensity value based on the length of "Intensity: "
  int offset = strlen(label) * 100; // Position calculation for the intensity value

  // Setting different LCD Color for different intensities
  if (strstr(intensity, "Low"))
  {
    lcd.SetTextColor(LCD_COLOR_GREEN);
    lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Low", LEFT_MODE);
  }
  else if (strstr(intensity, "Medium"))
  {
    lcd.SetTextColor(LCD_COLOR_BLUE);
    lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Medium", LEFT_MODE);
  }
  else if (strstr(intensity, "High"))
  {
    lcd.SetTextColor(LCD_COLOR_RED);
    lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"High", LEFT_MODE);
  }
}

// Clear a line on the LCD
void clear_line(uint8_t line)
{
  char empty_line[64]; // Adjusting the length based on the LCD width
  memset(empty_line, ' ', sizeof(empty_line) - 1);
  empty_line[sizeof(empty_line) - 1] = '\0';
  lcd.DisplayStringAt(0, line, (uint8_t *)empty_line, CENTER_MODE);
}

void no_tremor_detected()
{
  // Code to handle absence of detected tremor
  if (!message_displayed)
  {
    no_tremor_counter++;
    if (no_tremor_counter >= NO_TREMOR_TIME_THRESHOLD)
    {
      // Display "No Resting Tremor Detected" on the LCD
      lcd.SetTextColor(LCD_COLOR_WHITE);
      lcd.Clear(LCD_COLOR_BLACK);
      lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"No Resting Tremor", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Detected", CENTER_MODE);
      message_displayed = true; // Set flag to prevent re-display
    }
  }
}

int main()
{
  setup_lcd();              // Initial setup of the LCD
  spi.format(8, 3);         // Setup SPI format
  spi.frequency(1'000'000); // SPI frequency

  // Initialize FIR filters for each axis
  for (int i = 0; i < 3; i++) {
      arm_fir_init_f32(&S[i], NUM_TAPS, (float32_t *)&firCoeffs32, &firStateF32[i][0], BUFFER_SIZE);
  }

  // Configure control registers via SPI
  uint8_t write_buf[32], read_buf[32];
  write_buf[0] = CTRL_REG1;
  write_buf[1] = CTRL_REG1_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
  flags.wait_all(SPI_FLAG);

  write_buf[0] = CTRL_REG4;
  write_buf[1] = CTRL_REG4_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
  flags.wait_all(SPI_FLAG);

  while (true)
  {
    // Main loop for reading gyro data, applying the FIR filter, and checking for tremors
    uint16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz, filtered_gx, filtered_gy, filtered_gz;
    uint8_t write_buf[7] = {OUT_X_L | 0x80 | 0x40}, read_buf[7];
    spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Convert the data
    raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
    raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
    raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

    // Convert to physical values
    gx = ((float)raw_gx) * SCALING_FACTOR;
    gy = ((float)raw_gy) * SCALING_FACTOR;
    gz = ((float)raw_gz) * SCALING_FACTOR;

    // Apply FIR filter to each axis
    arm_fir_f32(&S[0], &gx, &filtered_gx, 0);
    arm_fir_f32(&S[1], &gy, &filtered_gy, 1);
    arm_fir_f32(&S[2], &gz, &filtered_gz, 2);

    // Add data to buffer and analyze
    add_to_buffer(filtered_gx, gyro_buffer_x, buffer_index_x);
    add_to_buffer(filtered_gy, gyro_buffer_y, buffer_index_y);
    add_to_buffer(filtered_gz, gyro_buffer_z, buffer_index_z);
    float average_x = compute_average(gyro_buffer_x);
    float average_y = compute_average(gyro_buffer_y);
    float average_z = compute_average(gyro_buffer_z);

    // Compute vector magnitude from averages
    float magnitude = sqrtf(average_x * average_x + average_y * average_y + average_z * average_z);


    
    // printf("Tremor detected: %4.5f degrees/s\n", average); // For serial Monitor check
    // For serial Monitor check
    if (detect_tremor(magnitude))
    {
      // printf("Tremor detected: %4.5f degrees/s\n", magnitude);
      no_tremor_counter = 0; // Reset counter on tremor detection
      message_displayed = false;
      log_event_time();
      float frequency = calculate_frequency();
      // printf("Tremor detected: %4.5f degrees/s, Frequency: %3.2f Hz\n", magnitude, frequency);
      if (frequency_index < FREQUENCY_BUFFER_SIZE)
      {
        frequency_buffer[frequency_index++] = frequency;
        if (frequency_index == FREQUENCY_BUFFER_SIZE)
        {
          frequency_buffer_full = true;
        }
      }
      if (frequency_buffer_full)
      {
     
        float avg_frequency = calculate_average_frequency();
        // printf("Tremor detected: %4.5f degrees/s\n, Avg Frequency: %3.2f Hz\n ", magnitude, avg_frequency); // For serial Monitor check
        // Code to limit frequency to 3-6Hz, handle detected tremor, including displaying on the LCD
        printf("Frequency: %3.2f Hz\n",avg_frequency); 
        if (avg_frequency >= 3.0f && avg_frequency <= 6.0f)
        {
          char buf[64];
          char intensity[64];
          printf("Tremor detected: %4.5f degrees/s, Frequency: %3.2f Hz\n", magnitude, avg_frequency); // For serial Monitor check
          snprintf(buf, sizeof(buf), "Tremor Freq: %3.2f Hz", avg_frequency);
          lcd.Clear(LCD_COLOR_BLACK); // Resetting the LCD screen to black
          // Setting Intensity based on tremor frequeny
          if (avg_frequency >= 3.0f && avg_frequency < 4.0f)
          {
            snprintf(intensity, sizeof(buf), "Intensity: Low");
          }
          else if (avg_frequency >= 4.0f && avg_frequency < 5.0f)
          {
            snprintf(intensity, sizeof(buf), "Intensity: Medium");
          }
          else
          {
            snprintf(intensity, sizeof(buf), "Intensity: High");
          }
          lcd.SetTextColor(LCD_COLOR_WHITE);
          clear_line(LINE(7));
          clear_line(LINE(8));
          lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Tremor Detection", CENTER_MODE);
          lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"System", CENTER_MODE);
          lcd.DisplayStringAt(0, LINE(7), (uint8_t *)buf, CENTER_MODE);
          display_intensity(intensity);
        }
        else
        {
          no_tremor_detected();
        }
        reset_frequency_buffer();  // Reset the frequency buffer
        // reset_tremor_event_tracking();  // Reset tremor event tracking
      }
    } else {
      no_tremor_detected();
      reset_frequency_buffer();  // Reset the frequency buffer
      // reset_tremor_event_tracking();  // Reset tremor event tracking
    }
    thread_sleep_for(1000 / SAMPLING_RATE); // Sleep to maintain the sampling rate
  }
}