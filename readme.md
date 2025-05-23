# Vibration Sensor Project

This project collects and visualizes sensor data from an IMU sensor. The Arduino-based firmware reads IMU sensor data and logs it to an SD card, publishes it via MQTT, and optionally sends the data to a PyQt-based user interface for real-time visualization.

## Features

- **Arduino Firmware**
  - Reads acceleration, gyroscope, angle, magnetometer, and temperature data.
  - Logs sensor data to an SD card with file rotation based on time/size.
  - Publishes sensor data over MQTT.
  - Uses an ESP32 (or compatible board) with I2C sensor connectivity.

- **PyQt User Interface**
  - Real-time oscilloscope-style display of sensor data.
  - MQTT client integration to receive and display sensor data.
  - Uses PyQt5, PyQtGraph, and paho-mqtt.

## Requirements

### For Arduino Firmware
- Arduino IDE or PlatformIO.
- ESP32 (or compatible Arduino board) with WiFi capability.
- [ArduinoJson](https://arduinojson.org/)
- I2C connection to a JY901 sensor (or compatible IMU)

### For PyQt UI
- Python 3.6 or later
- PyQt5
- pyqtgraph
- paho-mqtt

## Setup and Installation

### Arduino Firmware
1. **Install the Required Libraries**  
   Use the Arduino Library Manager or PlatformIO to install:
   - ArduinoJson


2. **Configure Sensor, SD Card, and Timing Settings**  
   Review the configuration constants in `imu_data_logger_sd.ino` under the `Config` namespace. This includes options for WiFi credentials, sensor I2C addresses, file settings, and sensor timing intervals. To adjust the sensor timing intervals, modify the following constants in the code:
   ```cpp
   // Timing intervals (ms)
   const unsigned long READ_INTERVAL = 100;  // Change sensor read interval (default: 100ms)
   const unsigned long SEND_INTERVAL = 1000; // Change data send interval (default: 1000ms)
   const unsigned long LOG_INTERVAL  = 1000; // Change data log interval (default: 1000ms)
   ```
   For example, to reduce the read interval to 50ms, update `READ_INTERVAL` as follows:
   ```cpp
   const unsigned long READ_INTERVAL = 50;  // Reduced read interval to 50ms
   ```

3. **Upload the Firmware**  
   Connect your board, select the proper board and port in your IDE, and upload the firmware.

### PyQt UI Application
1. **Install Python**  
   Ensure Python 3 is installed. Download from [python.org](https://www.python.org/downloads/) if needed.

2. **Set Up a Virtual Environment (Optional but Recommended)**
   Open Command Prompt in your project directory and run:
   ```bash
   python -m venv venv
   venv\Scripts\activate
   ```

3. **Install Dependencies**  
   In your virtual environment, install the required packages:
   ```bash
   pip install PyQt5 pyqtgraph paho-mqtt
   ```

4. **Run the PyQt Interface**  
   From your project folder, run:
   ```bash
   python mqtt_logger_ui.py
   ```

## Usage

- **Arduino Firmware:**  
  The firmware continuously reads sensor data, logs it to the SD card, and publishes the data over MQTT. Confirm connectivity via Serial Monitor and ensure the SD card initializes and logs correctly.

- **PyQt UI:**  
  The UI subscribes to the MQTT topic (adjust the topic in the code if necessary) and displays real-time sensor readings in both numerical and graphical forms.

## Troubleshooting

- **WiFi Connection Issues:**  
  Verify WiFi credentials and check network connectivity.

- **SD Card Errors:**  
  Ensure the SD card is properly connected and formatted.

- **MQTT Connection Issues:**  
  Verify broker connectivity and check that the correct MQTT topic is subscribed to.

## License

Specify your license information here.
