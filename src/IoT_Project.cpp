/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Windows PowerShell [& "C:\Program Files\mosquitto\mosquitto_sub" -h test.mosquitto.org -p 1883 -t outTopic/message]

// Include Particle Device OS APIs
#include "Particle.h"
#include "MQTT.h"
#include "Wire.h"
#include "string.h"

#define ICM20600_ADDR 0x69  // Default I²C address

// Calibration offsets
float ax_offset = 0;
float ay_offset = 0;
float az_offset = 0;

// Function to read raw accelerometer data
void readAccelerometer(int16_t &ax, int16_t &ay, int16_t &az) {
    Wire.beginTransmission(ICM20600_ADDR);
    Wire.write(0x3B);  // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(ICM20600_ADDR, 6);  // Request 6 bytes (X, Y, Z)
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
}

// Function to calibrate accelerometer
void calibrateAccelerometer() {
    int num_samples = 1000;
    int16_t ax, ay, az;
    long ax_sum = 0, ay_sum = 0, az_sum = 0;

    for (int i = 0; i < num_samples; i++) {
        readAccelerometer(ax, ay, az);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        delay(10);  // Delay to allow for stable readings
    }

    ax_offset = ax_sum / num_samples;
    ay_offset = ay_sum / num_samples;
    az_offset = az_sum / num_samples;
}

// MQTT client and callback setup
void callback(char* topic, byte* payload, unsigned int length);
MQTT client("test.mosquitto.org", 1883, callback);

// Callback function (called when a message is received)
void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];          // Create a buffer for the message
    memcpy(p, payload, length);  // Copy the payload into the buffer
    p[length] = NULL;            // Null-terminate the string
 
}

// // Compressing algorithm
// String compress(int val){
//   String newVal;

//   char data3;
//   if(val<0){
//     val = abs(val);
//     data3 = '-';
//   }
//   else{
//     data3 = '+';
//   }

//   char data1 = val & 0xFF;
//   char data2 = val >> 8;
//   newVal.concat(data1);
//   newVal.concat(data2);
//   newVal.concat(data3);

//   return newVal;
// } 

// Compressing algorithm
char* compress(int val){
  char* newVal;
  newVal = (char*)malloc(sizeof(char)*3);

  if(val<0){
    val = abs(val);
    newVal[2] = '-';
  }
  else{
    newVal[2] = '+';
  }

  newVal[0] = val & 0xFF;
  newVal[1] = val >> 8;

  return newVal;
} 

// Function to convert raw accelerometer data to m/s²
float convertToMS2(int16_t rawValue, float sensitivity) {
    return (rawValue / sensitivity) * 9.80665;
}


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// setup() runs once, when the device is first turned on
void setup() {
    // Put initialization like pinMode and begin functions here
    pinMode(A0, INPUT); // Force Sensor pin
    Serial.begin(9600);

    client.connect("myID");
    // Initialize MQTT connection
    if (client.isConnected()) {
        Serial.println("Connected to MQTT broker!");

    } else {
        Serial.println("Failed to connect to MQTT broker.");
    }
    Wire.begin();
    // Wake up the sensor
    Wire.beginTransmission(ICM20600_ADDR);
    Wire.write(0x6B);  // Power management register
    Wire.write(0x00);  // Wake up
    Wire.endTransmission();

    // Calibrate accelerometer
    calibrateAccelerometer();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
    // The core of your code will likely live here.

    int forceSensorAnalogMeasurement = analogRead(A0);
    String message;
    // char message[15]="ub"; //ub**+**+**+**+q

    // Read accelerometer data
    // Wire.beginTransmission(ICM20600_ADDR);
    // Wire.write(0x3B);  // Starting register for accelerometer data

    // Wire.endTransmission(false);
    // Wire.requestFrom(ICM20600_ADDR, 6);  // Request 6 bytes (X, Y, Z)

    // int16_t ax = (Wire.read() << 8) | Wire.read();
    // int16_t ay = (Wire.read() << 8) | Wire.read();
    // int16_t az = (Wire.read() << 8) | Wire.read();
    int16_t ax, ay, az;
    readAccelerometer(ax, ay, az);

    // Apply calibration offsets
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;

    // Convert raw values to m/s²
    float ax_ms2 = convertToMS2(ax, 16384.0); // Assuming ±2g range
    float ay_ms2 = convertToMS2(ay, 16384.0); // Assuming ±2g range
    float az_ms2 = convertToMS2(az, 16384.0); // Assuming ±2g range

    
    // // always out of heap memory, idk why
    // strcat(message,compress(forceSensorAnalogMeasurement));
    // strcat(message,compress(ax));
    // strcat(message,compress(ay));
    // strcat(message,compress(az));
    // strcat(message,"q");


    message.concat("s");
    message.concat(";");
    message.concat(forceSensorAnalogMeasurement);
    message.concat(";");
    message.concat(ax_ms2);
    message.concat(";");
    message.concat(ay_ms2);
    message.concat(";");
    message.concat(az_ms2);
    message.concat(";");
    message.concat("e");

    // data = "s;forcesensor;x;y;z;e"

    // Ensure the client stays connected
    if (client.isConnected()) {
        // Handle MQTT tasks (e.g., receiving messages)
        digitalWrite(D7, HIGH);
        // Periodically publish a message
        static unsigned long lastPublish = 0;
        if (millis() - lastPublish > 1000) { // Publish every 2 seconds
            client.publish("outTopic/message", message);
            // Serial.println("Published 'ciao' to 'outTopic/message'");
            lastPublish = millis();
        }
    } else {
        // Reconnect if the client is disconnected
        digitalWrite(D7, LOW);
        Serial.println("MQTT client disconnected. Reconnecting...");
        client.connect("myID");

    }

    client.loop(); 
}
