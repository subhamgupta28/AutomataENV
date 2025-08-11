/*****************************************************************************
* | File        :   tvoc_sensor.cpp
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :   Implementation of the TVOC sensor interface.
*----------------
* | This version:   V1.0
* | Date        :   2025-03-12
* | Info        :   Initial implementation of the TVOC sensor library.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
*****************************************************************************/

#include "Arduino.h"
#include "tvoc_sensor.h"

uint8_t rx_buf[11];               // Serial port receiving array (for storing incoming data)
uint8_t count_i = 0;

uint16_t co2 = 0, ch2o = 0, adc = 0;  // Variables to store CO2, formaldehyde (CH2O), and ADC values
float tvoc = 0;                      // Variable to store TVOC (Total Volatile Organic Compounds) value

// Command buffers for setting sensor modes and querying data
uint8_t active_buf[9] = {0xFE, 0x00, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0xB8}; // Command to set active mode
uint8_t stop_buf[9] = {0xFE, 0x00, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0xB9};   // Command to set query mode
uint8_t get_query_buf[9] = {0xFE, 0x00, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86}; // Command to query data in query mode

/**
 * Initialize the TVOC sensor hardware interface.
 * This function sets up the UART communication with the sensor and configures the reset and alarm pins.
 */
void tvoc_init()
{
  tvoc_uart.begin(115200, SERIAL_8N1, tvoc_rx, tvoc_tx); // Initialize UART communication at 115200 baud rate
  pinMode(tvoc_rst, OUTPUT); // Set the reset pin as an output
  pinMode(tvoc_alm, INPUT); // Set the alarm pin as an input
  digitalWrite(tvoc_rst, HIGH); // Pull the reset pin high to initialize the sensor
}

/********************************************************************** 
 * Function Name: uint8_t CRC_Check(uint8_t *Data, uint32_t Size)
 * Description: Calculate the checksum of the received data.
 * This function sums the elements of the data array to verify the integrity of the received data.
 **********************************************************************/
uint8_t CRC_Check(uint8_t *Data, uint32_t Size)
{
  uint32_t Count;
  uint8_t CrC = 0;
  uint8_t buf_crc[8]; // Array to store data for checksum calculation

  // Extract relevant data for checksum calculation based on the data size
  if (Size == 6)
    for (int i = 3; i < 9; i++) { // Extract data starting from index 3
      buf_crc[i - 3] = Data[i];
    }
  else
    for (int i = 1; i < 8; i++) { // Extract data starting from index 1
      buf_crc[i - 1] = Data[i];
    }

  // Calculate the checksum by summing the extracted data
  for (Count = 0; Count < Size; Count++) {
    CrC += buf_crc[Count];
  }
  return CrC; // Return the calculated checksum
}
const char* getCO2Status(uint16_t ppm) {
  if (ppm < 800) return "Good";
  if (ppm < 1200) return "Moderate";
  return "Poor";
}

const char* getCH2OStatus(uint16_t ppb) {
  if (ppb < 80) return "Good";
  if (ppb < 200) return "Moderate";
  return "Poor";
}

const char* getTVOCStatus(float ppm) {
  if (ppm < 0.3) return "Good";
  if (ppm < 0.6) return "Moderate";
  return "Poor";
}
/**
 * Get data from the TVOC sensor in active mode.
 * This function continuously reads data from the sensor and verifies its integrity using checksum.
 */
TvocSensorData tvoc_get_active_device_data() {
  TvocSensorData data = { false, 0, 0, 0.0,-1, "Unknown", "Unknown", "Unknown" };

  if (tvoc_uart.available() > 0) {
    if (tvoc_uart.peek() == 0xFE) {
      count_i = 0;
      rx_buf[count_i] = tvoc_uart.read();
    } else {
      rx_buf[count_i] = tvoc_uart.read();
    }
    count_i++;

    if (count_i > 10) {
      while (tvoc_uart.available()) {
        tvoc_uart.read();
      }

      count_i = 0;
      if (CRC_Check(rx_buf, 6) == rx_buf[9]) {
        data.co2 = (rx_buf[3] << 8) | rx_buf[4];
        data.ch2o = (rx_buf[5] << 8) | rx_buf[6];
        data.tvoc = ((rx_buf[7] << 8) | rx_buf[8]) / 1000.0;
        data.valid = true;

        // Assign status indicators
        data.co2_status = getCO2Status(data.co2);
        data.ch2o_status = getCH2OStatus(data.ch2o);
        data.tvoc_status = getTVOCStatus(data.tvoc);

        data.alarm = digitalRead(tvoc_alm);

       
      }
    }
  }

  delay(10);
  return data;
}



/**
 * Set the TVOC sensor to active mode.
 * In active mode, the sensor continuously sends data without needing a query.
 */
void tvoc_set_device_active_mode()
{
  while (tvoc_uart.read() >= 0); // Clear any remaining data in the UART buffer
  tvoc_uart.write(active_buf, 9); // Send the command to set active mode
}

/**
 * Set the TVOC sensor to query mode.
 * In query mode, the sensor only sends data when explicitly requested.
 */
void tvoc_set_device_query_mode()
{
  while (tvoc_uart.read() >= 0); // Clear any remaining data in the UART buffer
  tvoc_uart.write(stop_buf, 9); // Send the command to set query mode
}

/**
 * Get data from the TVOC sensor in query mode.
 * This function sends a query command and reads the response from the sensor.
 */
void tvoc_get_query_device_data()
{
  count_i = 0; // Reset the loop counter
  while (tvoc_uart.read() >= 0); // Clear any remaining data in the UART buffer
  tvoc_uart.write(get_query_buf, 9); // Send the query command to the sensor
  delay(15); // Wait for the sensor to respond

  // Read the response from the sensor
  while (tvoc_uart.available()) {
    rx_buf[count_i++] = tvoc_uart.read();
  }

  if (count_i > 10) {
    if (CRC_Check(rx_buf, 6) == rx_buf[9]) // Verify the checksum
    {
      // Extract sensor data from the received buffer
      tvoc = ((rx_buf[5] << 8) | rx_buf[6]) / 1000.0; // Combine two bytes and convert to TVOC value
      adc = (rx_buf[7] << 8) | rx_buf[8]; // Combine two bytes to get ADC value

      // Check if the alarm pin is triggered (TVOC exceeds 2ppm)
      if (digitalRead(tvoc_alm) == LOW)
        printf("TVOC has exceeded 2ppm\r\n");

      // Print the sensor data
      printf("ADC = %d TVOC = %0.3f\r\n", adc, tvoc);
    }
    else {
      printf("CRC_Check Fail!\r\n"); // Print error message if checksum verification fails
    }
  }
  delay(1000); // Delay before the next query
}