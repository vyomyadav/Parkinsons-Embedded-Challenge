# Parkinsons Embedded Challenge

This project implements a tremor detection system specifically designed to identify and analyze resting tremors associated with Parkinson's disease. Utilizing the ST DISCO-F429ZI board equipped with an LCD display and a gyroscope sensor, the system detects tremors, calculates their frequency, and visually displays the intensity on the LCD screen. The system is optimized to detect tremors within the frequency range of 3-6 Hz, providing a comprehensive tool for monitoring tremor severity.

### Video Link: https://youtu.be/XvMf5ppN2Xo

### Github Link: https://github.com/vyomyadav/Parkinsons-Embedded-Challenge

## Overview

The main components of the system include:
- **SPI Communication**: Reading data from the gyroscope.
- **FIR Filtering**: Smoothing the gyroscope data.
- **Magnitude Calculation**: Computing the vector magnitude from gyroscope data.
- **Tremor Detection**: Identifying tremors based on a threshold.
- **Frequency Calculation**: Determining the frequency of detected tremors.
- **Frequency Buffering**: Averaging recent tremor frequencies.
- **LCD Display**: Showing tremor intensity on the LCD.

## Components

- **LCD_DISCO_F429ZI**: LCD display driver for ST DISCO-F429ZI board.
- **CMSIS-DSP Library**: For digital signal processing.
- **mbed**: Framework for microcontroller programming.

## Required Configuration

### Setting Up the Tremor Threshold

The tremor threshold (`TREMOR_THRESHOLD`) is a critical parameter used to distinguish between actual tremor signals and background noise when the gyroscope is stationary. To ensure accurate tremor detection, follow these steps to configure the threshold:

**Note:** We have set the tremor threshold as 0.50 which has been verified as a valid threshold for our gyroscopes. If you are observing some noise on no movement. Then follow the below steps.

1. **Initial Setup**: Place the gyroscope sensor on a flat, stable surface to minimize any movement.

2. **Monitor Tremor Value**: Observe the tremor value displayed on the serial monitor when the gyroscope is stationary. Follow the line 291 in the code to figure out the observed tremor value. You will have to use a serial monitor for this.

3. **Adjust Threshold**:
   - If the observed tremor value exceeds the default threshold of 0.50, note this value.
   - Update the `TREMOR_THRESHOLD` to be slightly higher (about 0.1) than the observed tremor value to effectively filter out noise.
   
   Example: If the stationary tremor value is 0.55, set `TREMOR_THRESHOLD` to 0.65.

This configuration step ensures that the system accurately differentiates between actual tremors and background noise, enhancing the reliability of tremor detection. Adjusting the threshold based on real-world observations is essential for optimal system performance.


## Limitations

We encountered difficulties in getting the FFT (Fast Fourier Transform) function from the arm_math.h library to work as expected. As a result, we opted for an alternative method of frequency calculation by measuring the time intervals between detected tremor events.

### Advantages

 - Low computational overhead.
 - Real-time detection and response.

### Disadvantages
 - Less precise for complex or noisy signals.
 - Only captures the dominant frequency. 

### Frequency Calculation Method

 - **Log Tremor Events:** Each detected tremor event is time-stamped.
 - **Calculate Intervals:** The time intervals between consecutive tremor events are computed.
 - **Determine Frequency:** The frequency is calculated by taking the inverse of the average time interval (i.e., Frequency = 1 / Average Interval).

This approach provides a direct and practical means of determining tremor frequency without the complexities associated with FFT implementation.


## Steps Explained

### 1. SPI Communication

The gyroscope is interfaced with the microcontroller using SPI. The SPI communication is set up with a specific format and frequency, and control registers are configured to initialize the gyroscope.

### 2. FIR Filtering

FIR (Finite Impulse Response) filters are applied to the x, y, and z-axis data from the gyroscope to smooth the signals. The FIR filters are initialized with pre-defined coefficients and applied to the incoming data.

### 3. Magnitude Calculation

The magnitude of the vector from the x, y, and z values is calculated. This magnitude represents the overall motion detected by the gyroscope, combining the contributions from all three axes.

### 4. Tremor Detection

Tremors are detected by comparing the calculated magnitude against a predefined threshold. If the magnitude exceeds this threshold, a tremor is considered to have occurred.

### 5. Frequency Calculation

The frequency of tremors is calculated by logging the time of each detected tremor event and averaging the intervals between these events. This frequency is expressed in Hertz (Hz).

### 6. Frequency Buffering

A buffer of recent tremor frequencies is maintained to calculate an average frequency over a set number of samples. This helps in stabilizing the frequency measurement by smoothing out short-term fluctuations.

### 7. LCD Display

The intensity of the tremor is displayed on the LCD screen based on the average frequency calculated. The system categorizes the intensity into three levels: Low, Medium, and High, each represented by a different color on the LCD.


## Conclusion

This system effectively detects and analyzes resting tremors related to Parkinson's disease, which typically occur in the frequency range of 3-6 Hz. The intensity of these tremors is displayed on an LCD screen, providing a clear visual representation of the tremor severity. The system's modular structure allows for easy adjustments to thresholds and parameters based on real-world testing, ensuring accurate and reliable tremor detection and analysis.