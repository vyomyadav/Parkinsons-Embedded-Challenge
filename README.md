# Parkinsons Embedded Challenge

This project implements a tremor detection system using the ST DISCO-F429ZI board with an LCD display and a gyroscope sensor. The system detects tremors, calculates their frequency, and displays the intensity on the LCD screen.

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

1. **Initial Setup**: Place the gyroscope sensor on a flat, stable surface to minimize any movement.

2. **Monitor Tremor Value**: Observe the tremor value displayed on the serial monitor when the gyroscope is stationary.

3. **Adjust Threshold**:
   - If the observed tremor value exceeds the default threshold of 0.50, note this value.
   - Update the `TREMOR_THRESHOLD` to be slightly higher (about 0.1) than the observed tremor value to effectively filter out noise.
   
   Example: If the stationary tremor value is 0.55, set `TREMOR_THRESHOLD` to 0.65.

This configuration step ensures that the system accurately differentiates between actual tremors and background noise, enhancing the reliability of tremor detection. Adjusting the threshold based on real-world observations is essential for optimal system performance.


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

This system effectively detects and analyzes tremors using a gyroscope sensor, with results displayed on an LCD screen. The system's modular structure allows for easy adjustments to thresholds and parameters based on real-world testing.