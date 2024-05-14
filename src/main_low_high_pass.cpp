#include <mbed.h>
#include "drivers/LCD_DISCO_F429ZI.h" // Include the LCD display library for the ST DISCO-F429ZI board

#define WINDOW_SIZE 10 // Example window size, adjust as needed

// Define Regs & Configurations --> Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Second configure to set the DPS // page 33
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define CTRL_REG3 0x22 // page 32
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

#define FILTER_COEFFICIENT 0.2f // Adjust this value as needed
#define SAMPLING_RATE 100
#define NO_TREMOR_TIME_THRESHOLD (8 * SAMPLING_RATE) // 8 seconds converted to number of samples at given SAMPLING_RATE


LCD_DISCO_F429ZI lcd; // Create an LCD object

// EventFlags object declaration
EventFlags flags;

int no_tremor_counter = 0;      // Counter to track the duration of no tremor detection
bool message_displayed = false; // Flag to control the display of the no tremor message

// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}


// data ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

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
  //LOW = 3-4Hz
  if (strstr(intensity, "Low"))
  {
    lcd.SetTextColor(LCD_COLOR_GREEN);
    lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Low", LEFT_MODE);
  }
  //MEDIUM = 4-5Hz
  else if (strstr(intensity, "Medium"))
  {
    lcd.SetTextColor(LCD_COLOR_BLUE);
    lcd.DisplayStringAt(offset, LINE(9), (uint8_t *)"Medium", LEFT_MODE);
  }
  //HIGH = 5-6Hz
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

// Code to handle absence of detected tremor
void no_tremor_detected()
{
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



int main() {
    setup_lcd();
    //spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    //interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);
    
    //spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers --> spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //(polling for\setting) data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // Example 2: LPF definitions
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;

    // Example 3: HPF definitions
    // use with the example 2 definitions
    float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f;

    Timer timer;
    timer.start();
    int threshold_cross_count = 0;

    while (1) {
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        flags.wait_all(DATA_READY_FLAG);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        // Example 2: Apply Simple low-pass filter
        filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

        // Example 3: Apply simple high-pass filter with the lpf (by eliminating low freq elements)
        // to be used with example 2 (together)
        high_pass_gx = gx - filtered_gx;
        high_pass_gy = gy - filtered_gy;
        high_pass_gz = gz - filtered_gz;

        printf(">x_axis_high:%4.5f\n", high_pass_gx);
        printf(">y_axis_high:%4.5f\n", high_pass_gy);
        printf(">z_axis_high:%4.5f\n", high_pass_gz);

        // Detect threshold-crossings
        if ((high_pass_gx < -0.1) || (high_pass_gx > 0.1)) {
            threshold_cross_count++;
        }

        // Check the timer to count for one second
        if (timer.read_ms() >= 1000) {
            int avg_frequency = threshold_cross_count;  // Full oscillations

            if (avg_frequency >= 3 && avg_frequency <= 6) {

                char buf[64];
                char intensity[64];
                printf("Oscillations are between 3 and 6 Hz: %d Hz\n", avg_frequency);

                snprintf(buf, sizeof(buf), "Tremor Freq: %d Hz", avg_frequency);
                lcd.Clear(LCD_COLOR_BLACK); // Resetting the LCD screen to black
                // Setting Intensity based on tremor frequeny
                if (avg_frequency >= 3 && avg_frequency < 4)
                {
                    snprintf(intensity, sizeof(buf), "Intensity: Low");
                }
                else if (avg_frequency >= 4 && avg_frequency < 5)
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

            } else {
                // Function to handle the absence of detected tremor
                no_tremor_detected();
            }
            
            // Reset for the next second
            threshold_cross_count = 0;
            timer.reset();
        }

        thread_sleep_for(1000 / SAMPLING_RATE);
    }
}


