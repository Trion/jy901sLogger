// autoformat with copilot github (codeformator)
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include "mongoose_glue.h"
#include "time.h"


// in Dataprocessor function (in )
// chart vaue (g value )    float gForce = sqrt(data.accel.x * data.accel.x + data.accel.y * data.accel.y + data.accel.z * data.accel.z)/ 9.81f;
const char* value;
const char* log_value;

extern struct mg_mgr g_mgr;
extern mg_connection* g_mqtt_conn;

// Configuration Constants
namespace Config {
const char* SSID = "Innovatic IOTHouse";
const char* PASSWORD = "saiIIH2024";
const char* NTPSERVER = "pool.ntp.org";
const long GMTOFFSET_SEC = 8 * 3600;  // UTC // please fix utc office
const int DAYLIGHTOFFSET_SEC = 0;

const uint8_t JY901_ADDRESS = 0x50;  // Default I2C address

// Register addresses
enum Register {
  ACCEL = 0x34,
  GYRO = 0x37,
  ANGLE = 0x3D,
  MAG = 0x3A,
  TEMP = 0x40
};

const uint8_t SD_CS_PIN = 5;
const size_t VIBRATION_HISTORY_SIZE = 100;
const size_t MAX_ACC_POINTS = 10;

// Timing intervals (ms)
const unsigned long READ_INTERVAL = 100;  // change read interval
const unsigned long SEND_INTERVAL = 1000;
const unsigned long LOG_INTERVAL = 1000;

// Log file settings
const unsigned long MAX_LOG_FILE_TIME = 3600000;          // 1 hour
const unsigned long MAX_LOG_FILE_SIZE = 5 * 1024 * 1024;  // 5MB
}

// Sensor Data Structure using mongoose_glue.h types
struct SensorData {
  struct acceleration accel;
  struct gyo gyro;
  struct angle angle;
  struct {
    float x, y, z;
  } mag;
  float temperature;
  time_t timestamp;
};

class VibrationHistory {
private:
  float x[Config::VIBRATION_HISTORY_SIZE];
  float y[Config::VIBRATION_HISTORY_SIZE];
  float z[Config::VIBRATION_HISTORY_SIZE];
  size_t index = 0;

public:
  void update(float xVal, float yVal, float zVal) {
    x[index] = xVal;
    y[index] = yVal;
    z[index] = zVal;
    index = (index + 1) % Config::VIBRATION_HISTORY_SIZE;
  }

  void getRecent(size_t count, float* xOut, float* yOut, float* zOut) {
    for (size_t i = 0; i < count; i++) {
      size_t idx = (index - count + i + Config::VIBRATION_HISTORY_SIZE) % Config::VIBRATION_HISTORY_SIZE;
      xOut[i] = x[idx];
      yOut[i] = y[idx];
      zOut[i] = z[idx];
    }
  }
};

class CircularBuffer {
private:
  struct DataPoint {
    uint32_t timestamp;
    float value;
  };

  DataPoint buffer[Config::MAX_ACC_POINTS];
  size_t index = 0;
  bool filled = false;

public:
  void add(uint32_t ts, float val) {
    buffer[index] = { ts, val };
    index = (index + 1) % Config::MAX_ACC_POINTS;
    if (index == 0) filled = true;
  }

  void toJsonArray(JsonArray& array) {
    size_t count = filled ? Config::MAX_ACC_POINTS : index;
    size_t startIdx = filled ? index : 0;

    for (size_t i = 0; i < count; i++) {
      size_t idx = (startIdx + i) % Config::MAX_ACC_POINTS;
      JsonArray point = array.createNestedArray();
      point.add(buffer[idx].timestamp);
      point.add(buffer[idx].value);
    }
  }
};

// for event page (log save in mongooze wizard)
class LogEventBuffer {
private:
  struct LogDataPoint {
    uint32_t timestamp;
    float x;
    float y;
    float z;
    String d;
  };

  LogDataPoint buffer[Config::MAX_ACC_POINTS];  // max data point default is 10
  size_t index = 0;
  bool filled = false;

public:
  void add(uint32_t ts, float x, float y, float z, String d) {
    buffer[index] = { ts, x, y, z, d };
    index = (index + 1) % Config::MAX_ACC_POINTS;
    if (index == 0) filled = true;
  }

  void toJsonArray(JsonArray& array) {
    size_t count = filled ? Config::MAX_ACC_POINTS : index;
    size_t startIdx = filled ? index : 0;

    for (size_t i = 0; i < count; i++) {
      size_t idx = (startIdx + i) % Config::MAX_ACC_POINTS;
      JsonObject obj = array.createNestedObject();
      obj["timestamp"] = buffer[idx].timestamp;
      obj["x"] = buffer[idx].x;
      obj["y"] = buffer[idx].y;
      obj["z"] = buffer[idx].z;
      obj["d"] = buffer[idx].d;
    }
  }

  const char* toJsonString() {
    StaticJsonDocument<1024> doc;
    JsonArray array = doc.to<JsonArray>();
    toJsonArray(array);
    String jsonStr;
    serializeJson(doc, jsonStr);
    return strdup(jsonStr.c_str());
  }
};

class SDLogger {
private:
  bool available = false;
  String currentFile;
  unsigned long fileStartTime = 0;

public:
  void begin() {
    available = SD.begin(Config::SD_CS_PIN);
    if (available) {
      Serial.println("SD card initialized successfully");
      if (!SD.exists("/logs")) {
        SD.mkdir("/logs");
      }
      createNewFile();
    } else {
      Serial.println("SD card initialization failed");
    }
  }

  void createNewFile() {
    if (!available) return;

    fileStartTime = millis();
    char filename[32];
    sprintf(filename, "/logs/log_%lu.csv", time(nullptr));
    currentFile = String(filename);

    File file = SD.open(currentFile, FILE_WRITE);
    if (file) {
      file.println("timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,mag_x,mag_y,mag_z,temperature");
      file.close();
      Serial.print("Created new log file: ");
      Serial.println(currentFile);
    } else {
      Serial.println("Failed to create log file");
    }
  }

  void logData(const SensorData& data, uint32_t ts) {
    if (!available || currentFile == "") return;

    File file = SD.open(currentFile, FILE_APPEND);
    if (file) {
      file.print(ts);
      file.print(",");
      file.print(data.accel.x);
      file.print(",");
      file.print(data.accel.y);
      file.print(",");
      file.print(data.accel.z);
      file.print(",");
      file.print(data.gyro.x);
      file.print(",");
      file.print(data.gyro.y);
      file.print(",");
      file.print(data.gyro.z);
      file.print(",");
      file.print(data.angle.roll);
      file.print(",");
      file.print(data.angle.pitch);
      file.print(",");
      file.print(data.angle.yaw);
      file.print(",");
      file.print(data.mag.x);
      file.print(",");
      file.print(data.mag.y);
      file.print(",");
      file.print(data.mag.z);
      file.print(",");
      file.println(data.temperature);

      file.close();
    } else {
      Serial.println("Failed to open log file for writing");
    }
  }

  // new file create or other
  void checkRotation() {
    if (!available || currentFile == "") return;

    bool needNewFile = false;

    // Time-based rotation
    if (millis() - fileStartTime >= Config::MAX_LOG_FILE_TIME) {
      needNewFile = true;
    }

    // Size-based rotation
    if (!needNewFile) {
      File file = SD.open(currentFile);
      if (file && file.size() >= Config::MAX_LOG_FILE_SIZE) {
        needNewFile = true;
      }
      if (file) file.close();
    }

    if (needNewFile) {
      createNewFile();
    }
  }

  bool isAvailable() const {
    return available;
  }
};

// jy sensor reading (reformat with github copilot)
class JY901Sensor {
private:
  bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(Config::JY901_ADDRESS);
    Wire.write(reg);
    byte error = Wire.endTransmission(false);

    if (error != 0) {
      Serial.print("I2C error: ");
      Serial.println(error);
      return false;
    }

    byte bytesRead = Wire.requestFrom(Config::JY901_ADDRESS, length);
    if (bytesRead != length) {
      Serial.print("Requested ");
      Serial.print(length);
      Serial.print(" bytes, got ");
      Serial.println(bytesRead);
      return false;
    }

    for (uint8_t i = 0; i < length; i++) {
      buffer[i] = Wire.read();
    }

    return true;
  }

public:
  bool readAcceleration(SensorData& data) {
    uint8_t buffer[6];
    if (!readRegisters(Config::ACCEL, buffer, 6)) return false;

    int16_t rawX = (buffer[1] << 8) | buffer[0];
    int16_t rawY = (buffer[3] << 8) | buffer[2];
    int16_t rawZ = (buffer[5] << 8) | buffer[4];

    data.accel.x = (float)rawX / 32768.0f * 16.0f * 9.8f;
    data.accel.y = (float)rawY / 32768.0f * 16.0f * 9.8f;
    data.accel.z = (float)rawZ / 32768.0f * 16.0f * 9.8f;

    return true;
  }

  bool readGyroscope(SensorData& data) {
    uint8_t buffer[6];
    if (!readRegisters(Config::GYRO, buffer, 6)) return false;

    int16_t rawX = (buffer[1] << 8) | buffer[0];
    int16_t rawY = (buffer[3] << 8) | buffer[2];
    int16_t rawZ = (buffer[5] << 8) | buffer[4];

    data.gyro.x = (float)rawX / 32768.0f * 2000.0f;
    data.gyro.y = (float)rawY / 32768.0f * 2000.0f;
    data.gyro.z = (float)rawZ / 32768.0f * 2000.0f;

    return true;
  }

  bool readAngle(SensorData& data) {
    uint8_t buffer[6];
    if (!readRegisters(Config::ANGLE, buffer, 6)) return false;

    int16_t rawRoll = (buffer[1] << 8) | buffer[0];
    int16_t rawPitch = (buffer[3] << 8) | buffer[2];
    int16_t rawYaw = (buffer[5] << 8) | buffer[4];

    data.angle.roll = (float)rawRoll / 32768.0f * 180.0f;
    data.angle.pitch = (float)rawPitch / 32768.0f * 180.0f;
    data.angle.yaw = (float)rawYaw / 32768.0f * 180.0f;

    return true;
  }

  bool readMagnetometer(SensorData& data) {
    uint8_t buffer[6];
    if (!readRegisters(Config::MAG, buffer, 6)) return false;

    int16_t rawX = (buffer[1] << 8) | buffer[0];
    int16_t rawY = (buffer[3] << 8) | buffer[2];
    int16_t rawZ = (buffer[5] << 8) | buffer[4];

    data.mag.x = (float)rawX;
    data.mag.y = (float)rawY;
    data.mag.z = (float)rawZ;

    return true;
  }

  bool readTemperature(SensorData& data) {
    uint8_t buffer[2];
    if (!readRegisters(Config::TEMP, buffer, 2)) return false;

    int16_t rawTemp = (buffer[1] << 8) | buffer[0];
    data.temperature = (float)rawTemp / 100.0f;

    return true;
  }

  bool testConnection() {
    Wire.beginTransmission(Config::JY901_ADDRESS);
    byte error = Wire.endTransmission();
    return error == 0;
  }
};

// for chart
class DataProcessor {
private:
  CircularBuffer accelBuffer;
  LogEventBuffer logeventbuffer;
  char accelJson[512];
  char angleJson[512];
  String log_sensor_data;

public:
  void processAcceleration(const SensorData& data) {
    float gForce = sqrt(
                     data.accel.x * data.accel.x + data.accel.y * data.accel.y + data.accel.z * data.accel.z)
                   / 9.81f;

    accelBuffer.add(data.timestamp, gForce);

    StaticJsonDocument<512> doc;
    JsonArray root = doc.to<JsonArray>();
    accelBuffer.toJsonArray(root);
    serializeJson(doc, accelJson, sizeof(accelJson));

    Serial.print("g_force: ");
    Serial.println(gForce);
    Serial.print("acc_json: ");
    Serial.println(accelJson);
    value = accelJson;
  }

  void processAngle(const SensorData& data) {
    StaticJsonDocument<512> doc;
    JsonArray root = doc.to<JsonArray>();
    JsonArray point = root.createNestedArray();
    point.add(data.timestamp);
    point.add(data.angle.roll);
    serializeJson(doc, angleJson, sizeof(angleJson));

    Serial.print("angle_value: ");
    Serial.println(data.angle.roll);
    Serial.print("angle_json: ");
    Serial.println(angleJson);
  }

  void processLogEvent(uint32_t ts, float x, float y, float z, String d) {
    logeventbuffer.add(ts, x, y, z, d);
    const char* event_log = logeventbuffer.toJsonString();
    Serial.println("Event LOG : ");
    Serial.println(event_log);

    log_value = event_log;
    free((void*)event_log);
  }

  const char* getAccelJson() const {
    return accelJson;
  }
  const char* getAngleJson() const {
    return angleJson;
  }
};

namespace SensorSystem {
class NetworkManager {
private:
  WiFiClient clients[5];
  bool connected[5] = { false };

public:
  void connectToWiFi() {
    WiFi.begin(Config::SSID, Config::PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }

    Serial.println("-----------------------------");
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  void syncTime() {
    configTime(Config::GMTOFFSET_SEC, Config::DAYLIGHTOFFSET_SEC, Config::NTPSERVER);
    delay(2000);
    Serial.print("Unix timestamp (UTC): ");
    Serial.println(time(nullptr));
    Serial.println("-----------------------------");
  }

  //optional features not used in this code(testing)
  String sendData(const SensorData& data, const VibrationHistory& vibHistory) {
    StaticJsonDocument<1024> doc;

    JsonObject accel = doc.createNestedObject("accel");
    accel["x"] = data.accel.x;
    accel["y"] = data.accel.y;
    accel["z"] = data.accel.z;

    JsonObject gyro = doc.createNestedObject("gyro");
    gyro["x"] = data.gyro.x;
    gyro["y"] = data.gyro.y;
    gyro["z"] = data.gyro.z;

    JsonObject angle = doc.createNestedObject("angle");
    angle["roll"] = data.angle.roll;
    angle["pitch"] = data.angle.pitch;
    angle["yaw"] = data.angle.yaw;

    JsonObject mag = doc.createNestedObject("mag");
    mag["x"] = data.mag.x;
    mag["y"] = data.mag.y;
    mag["z"] = data.mag.z;

    doc["temp"] = data.temperature;

    float recentVibX[50], recentVibY[50], recentVibZ[50];
    //vibHistory.getRecent(50, recentVibX, recentVibY, recentVibZ);

    JsonArray vibX = doc.createNestedArray("vibX");
    JsonArray vibY = doc.createNestedArray("vibY");
    JsonArray vibZ = doc.createNestedArray("vibZ");

    for (int i = 0; i < 50; i++) {
      vibX.add(recentVibX[i]);
      vibY.add(recentVibY[i]);
      vibZ.add(recentVibZ[i]);
    }

    String jsonString;
    serializeJson(doc, jsonString);
    Serial.println(jsonString);
    return jsonString;
    // for (int i = 0; i < 5; i++) {
    //   if (connected[i]) {
    //     clients[i].println("data: " + jsonString + "\n");
    //   }
    // }
  }

  void publish(const char* topic, const char* message) {
    if (g_mqtt_conn != NULL) {
      struct mg_mqtt_opts opts;
      memset(&opts, 0, sizeof(opts));
      opts.topic = mg_str(topic);
      opts.message = mg_str(message);
      opts.qos = 1;

      mg_mqtt_pub(g_mqtt_conn,&opts);
      MG_DEBUG(("Publised  to topic [%s]: %s",topic,message));
    }else {
      MG_DEBUG(("Publised  to topic [%s]: %s",topic,message));
    }
  }

  // with pyqtui (test version )
  void sendToPyQt(const SensorData& data) {
    // Send acceleration
    Serial.print("$ACCEL,");
    Serial.print(data.accel.x, 4);
    Serial.print(",");
    Serial.print(data.accel.y, 4);
    Serial.print(",");
    Serial.print(data.accel.z, 4);
    Serial.println("*");

    // Send gyroscope
    Serial.print("$GYRO,");
    Serial.print(data.gyro.x, 4);
    Serial.print(",");
    Serial.print(data.gyro.y, 4);
    Serial.print(",");
    Serial.print(data.gyro.z, 4);
    Serial.println("*");

    // Send angle
    Serial.print("$ANGLE,");
    Serial.print(data.angle.roll, 4);
    Serial.print(",");
    Serial.print(data.angle.pitch, 4);
    Serial.print(",");
    Serial.print(data.angle.yaw, 4);
    Serial.println("*");

    // Send temperature
    Serial.print("$TEMP,");
    Serial.print(data.temperature, 2);
    Serial.println("*");
  }
};
}



// Global instances
JY901Sensor imu;
SDLogger logger;
SensorSystem::NetworkManager network;
DataProcessor processor;
VibrationHistory vibrationHistory;
SensorData currentData;

// Timing variables
unsigned long lastReadTime = 0;
unsigned long lastSendTime = 0;
unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("data logger v1.1");
  Serial.println("------------------------");

  // Initialize SD card
  logger.begin();

  // Connect to WiFi and sync time
  network.connectToWiFi();
  network.syncTime();

  // Check IMU connection
  if (imu.testConnection()) {
    Serial.println("sensor found on I2C bus");
  } else {
    Serial.println("Error connecting to JY901 - check connections");
  }

  // Initialize Mongoose
  mongoose_init();

  // Initialize glue data
  glue_set_acceleration(&currentData.accel);
  glue_set_gyo(&currentData.gyro);
  glue_set_angle(&currentData.angle);
}

int log_count = 0;

void loop() {
  mongoose_poll();
  currentData.timestamp = time(nullptr);

  unsigned long currentTime = millis();

  // Read sensor data
  if (currentTime - lastReadTime >= Config::READ_INTERVAL) {
    lastReadTime = currentTime;
    log_count++;
    imu.readAcceleration(currentData);
    imu.readGyroscope(currentData);
    imu.readAngle(currentData);
    imu.readMagnetometer(currentData);
    imu.readTemperature(currentData);

    vibrationHistory.update(currentData.accel.x, currentData.accel.y, currentData.accel.z);

    processor.processAcceleration(currentData);
    processor.processAngle(currentData);

    // processor.processLogEvent(currentData.timestamp, currentData.accel.x, currentData.accel.y, currentData.accel.z, "accel");
    // processor.processLogEvent(currentData.timestamp, currentData.gyro.x, currentData.gyro.y, currentData.gyro.z, "gyo");
    // processor.processLogEvent(currentData.timestamp, currentData.angle.roll, currentData.angle.pitch, currentData.angle.yaw, "gyo");

    // Update glue data
    glue_set_acceleration(&currentData.accel);
    glue_set_gyo(&currentData.gyro);
    glue_set_angle(&currentData.angle);
    glue_update_state();
  }

  // Send data to clients (pyqtui test version)
  if (currentTime - lastSendTime >= Config::SEND_INTERVAL) {
    lastSendTime = currentTime;
    String raw_data = network.sendData(currentData, vibrationHistory);
    network.publish("device",String(raw_data).c_str());

    //test data with pyqtui  file not using mongoose wizard
    //network.sendToPyQt(currentData);
  }

  // Log data to SD card
  if (logger.isAvailable() && (currentTime - lastLogTime >= Config::LOG_INTERVAL)) {
    lastLogTime = currentTime;
    logger.logData(currentData, currentData.timestamp);
    logger.checkRotation();  // check log file create or not
  }
}

extern "C" int lwip_hook_ip6_input(struct pbuf* p, struct netif* inp) __attribute__((weak));
extern "C" int lwip_hook_ip6_input(struct pbuf* p, struct netif* inp) {
  if (ip6_addr_isany_val(inp->ip6_addr[0].u_addr.ip6)) {
    pbuf_free(p);
    return 1;
  }
  return 0;
}