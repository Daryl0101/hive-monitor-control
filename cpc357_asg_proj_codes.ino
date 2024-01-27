#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ESP32Servo.h>
#include "environ.h"

const int VPERIPHERAL_DELAY_TIME = 15 * 60 * 1000;
const int SENSOR_DELAY_TIME = 2000;
const int CONTROL_PIN = 11;
const int DHT_PIN = 42;
const int DHT_TYPE = DHT11;
const int TEMPERATURE_THRESHOLD = 30;
const int HUMIDITY_THRESHOLD = 76;
// int TEMPERATURE_THRESHOLD[3] = {30, 35, 40};
// int HUMIDITY_THRESHOLD[3] = {70, 80, 90};

const int IR_SENSOR_1_PIN = 14;
const int IR_SENSOR_2_PIN = 21;
const int PASS_FIRST_IR_MAX_TIME = 5000;
const int PASS_SECOND_IR_MAX_TIME = 0;
const int COLOR_SENSOR_S0_PIN = 47;
const int COLOR_SENSOR_S1_PIN = 48;
const int COLOR_SENSOR_S2_PIN = 38;
const int COLOR_SENSOR_S3_PIN = 39;
const int COLOR_SENSOR_OUTPUT_PIN = 40;
const int COLOR_SENSOR_SETTLE_TIME = 100; // Settle time per color sensor, must be less than PASS_FIRST_IR_MAX_TIME
const int RED_LOWER = 40;
const int RED_UPPER = 700;
const int GREEN_LOWER = 60;
const int GREEN_UPPER = 610;
const int BLUE_LOWER = 30;
const int BLUE_UPPER = 500;

const int RAIN_SENSOR_DIGITAL_PIN = A4;
const int RAIN_SENSOR_ANALOG_PIN = A5;
const int LDR_SENSOR_ANALOG_PIN = A2;
const int RAIN_INTENSITY_THRESHOLD = 50;
const int LIGHT_INTENSITY_THRESHOLD = 30;

const int DC_MOTOR_PIN = A0;
// int DC_MOTOR_FAN_SPEED[4] = {0, 64, 128, 255};
const int SERVO_MOTOR_PIN = A1;
const int SERVO_MOTOR_MOVING_TIME = 3000;

DHT dht(DHT_PIN, DHT_TYPE);
Servo myServo;
WiFiClient espClient;
PubSubClient client(espClient);
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT server");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_MOTOR_PIN);
  myServo.write(0);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  pinMode(CONTROL_PIN, OUTPUT);
  pinMode(DC_MOTOR_PIN, OUTPUT);
  pinMode(IR_SENSOR_1_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_2_PIN, INPUT_PULLUP);
  pinMode(COLOR_SENSOR_S0_PIN, OUTPUT);
  pinMode(COLOR_SENSOR_S1_PIN, OUTPUT);
  pinMode(COLOR_SENSOR_S2_PIN, OUTPUT);
  pinMode(COLOR_SENSOR_S3_PIN, OUTPUT);

  // Setting frequency scaling of color sensor to 20%
  digitalWrite(COLOR_SENSOR_S0_PIN, HIGH);
  digitalWrite(COLOR_SENSOR_S1_PIN, LOW);
}

void ingressEgressCounterFunction(){
  static char *gress_types[3] = {"NA", "Ingress", "Egress"};
  static char *current_gress_type = gress_types[0];
  static unsigned long pass_first_ir_time = 0;
  static unsigned long pass_second_ir_time = 0;
  static bool is_second_pin_triggered = false;
  static bool is_color_data_taken = false;
  static int rgb[3] = {0, 0, 0};
  static bool rgb_test_state[3] = {false, false, false};

  // Delay next detection to reduce false alarm
  if (is_second_pin_triggered){
    if (millis() - pass_second_ir_time > PASS_SECOND_IR_MAX_TIME){
      current_gress_type = gress_types[0];
      pass_first_ir_time = 0;
      pass_second_ir_time = 0;
      for (int i=0; i<sizeof(rgb_test_state) / sizeof(rgb_test_state[0]); i++) {
        rgb_test_state[i] = false;
        rgb[i] = 0;
      }
      is_color_data_taken = false;
      is_second_pin_triggered = false;
    }
  }
  else {
    // Ingress
    if (current_gress_type == gress_types[1]) {
      if (millis() - pass_first_ir_time < PASS_FIRST_IR_MAX_TIME) {
        // check if another IR sensor is triggered
        if (!is_second_pin_triggered) is_second_pin_triggered = !digitalRead(IR_SENSOR_2_PIN);
        if (!rgb_test_state[0]){
          // Read RED (R) color
          digitalWrite(COLOR_SENSOR_S2_PIN, LOW);
          digitalWrite(COLOR_SENSOR_S3_PIN, LOW);
          rgb[0] = map(pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW), RED_LOWER, RED_UPPER, 255, 0);
          rgb_test_state[0] = true;
        }
        else if (millis() - pass_first_ir_time >= COLOR_SENSOR_SETTLE_TIME * 1  && !rgb_test_state[1]){
          // Read GREEN (G) color
          digitalWrite(COLOR_SENSOR_S2_PIN, HIGH);
          digitalWrite(COLOR_SENSOR_S3_PIN, HIGH);
          rgb[1] = map(pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW), GREEN_LOWER, GREEN_UPPER, 255, 0);
          rgb_test_state[1] = true;
        }
        else if (millis() - pass_first_ir_time >= COLOR_SENSOR_SETTLE_TIME * 2  && !rgb_test_state[2]){
          // Read BLUE (B) color
          digitalWrite(COLOR_SENSOR_S2_PIN, LOW);
          digitalWrite(COLOR_SENSOR_S3_PIN, HIGH);
          rgb[2] = map(pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW), BLUE_LOWER, BLUE_UPPER, 255, 0);
          rgb_test_state[2] = true;
          is_color_data_taken = true;
        }

        if (is_second_pin_triggered && is_color_data_taken){
          pass_second_ir_time = millis();
          Serial.println("Ingress Detected!!");
          char payload[1000];
          // unsigned long time_difference;
          // if (pass_second_ir_time >= pass_first_ir_time) time_difference = pass_second_ir_time - pass_first_ir_time;
          // else time_difference = (ULONG_MAX - pass_first_ir_time) + pass_second_ir_time + 1;
          // float time_seconds = time_difference / 1000.0;
          sprintf(payload, "Type: %s, Interval (s): %.2f, Color (R, G, B): (%d, %d, %d)", current_gress_type, (pass_second_ir_time - pass_first_ir_time) / 1000.0, rgb[0], rgb[1], rgb[2]);
          client.publish(MQTT_TOPIC_MOVEMENT, payload);
        }
      }
      // Reset if second IR sensor is not triggered within allowed time interval
      else {
        current_gress_type = gress_types[0];
        pass_first_ir_time = 0;
        for (int i=0; i<sizeof(rgb_test_state) / sizeof(rgb_test_state[0]); i++) {
          rgb_test_state[i] = false;
          rgb[i] = 0;
        }
        is_color_data_taken = false;
        is_second_pin_triggered = false;
      }
    }
    // Egress
    else if (current_gress_type == gress_types[2]) {
      if (millis() - pass_first_ir_time < PASS_FIRST_IR_MAX_TIME) {
        // check if another IR sensor is triggered
        if (!is_second_pin_triggered) is_second_pin_triggered = !digitalRead(IR_SENSOR_1_PIN);
        if (!rgb_test_state[0]){
          // Read RED (R) color
          digitalWrite(COLOR_SENSOR_S2_PIN, LOW);
          digitalWrite(COLOR_SENSOR_S3_PIN, LOW);
          rgb[0] = map(pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW), RED_LOWER, RED_UPPER, 255, 0);
          rgb_test_state[0] = true;
        }
        else if (millis() - pass_first_ir_time >= COLOR_SENSOR_SETTLE_TIME * 2  && !rgb_test_state[1]){
          // Read GREEN (G) color
          digitalWrite(COLOR_SENSOR_S2_PIN, HIGH);
          digitalWrite(COLOR_SENSOR_S3_PIN, HIGH);
          rgb[1] = map(pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW), GREEN_LOWER, GREEN_UPPER, 255, 0);
          rgb_test_state[1] = true;
        }
        else if (millis() - pass_first_ir_time >= COLOR_SENSOR_SETTLE_TIME * 3  && !rgb_test_state[2]){
          // Read BLUE (B) color
          digitalWrite(COLOR_SENSOR_S2_PIN, LOW);
          digitalWrite(COLOR_SENSOR_S3_PIN, HIGH);
          rgb[2] = map(pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW), BLUE_LOWER, BLUE_UPPER, 255, 0);
          rgb_test_state[2] = true;
          is_color_data_taken = true;
        }

        if (is_second_pin_triggered && is_color_data_taken){
          pass_second_ir_time = millis();
          Serial.println("Egress Detected!!");
          char payload[1000];
          // unsigned long time_difference;
          // if (pass_second_ir_time >= pass_first_ir_time) time_difference = pass_second_ir_time - pass_first_ir_time;
          // else time_difference = (ULONG_MAX - pass_first_ir_time) + pass_second_ir_time + 1;
          // float time_seconds = time_difference / 1000.0;
          sprintf(payload, "Type: %s, Interval (s): %.2f, Color (R, G, B): (%d, %d, %d)", current_gress_type, (pass_second_ir_time - pass_first_ir_time) / 1000.0, rgb[0], rgb[1], rgb[2]);
          client.publish(MQTT_TOPIC_MOVEMENT, payload);
        }
      }
      else {
        current_gress_type = gress_types[0];
        pass_first_ir_time = 0;
        for (int i=0; i<sizeof(rgb_test_state) / sizeof(rgb_test_state[0]); i++) {
          rgb_test_state[i] = false;
          rgb[i] = 0;
        }
        is_color_data_taken = false;
        is_second_pin_triggered = false;
      }
    }
    // No gress type
    else {
      if (!digitalRead(IR_SENSOR_1_PIN)){
        current_gress_type = gress_types[1];
        pass_first_ir_time = millis();
      }
      else if (!digitalRead(IR_SENSOR_2_PIN)){
        current_gress_type = gress_types[2];
        pass_first_ir_time = millis();
      }
    }
  }
}

bool hiveCoolingFunction(){
  static bool is_dht_begin = false;
  static bool is_start_time_set = false;
  static unsigned long start_time = millis();
  if (!is_start_time_set) {
    start_time = millis();
    is_start_time_set = true;
  }
  if (!is_dht_begin){
    dht.begin();
    is_dht_begin = true;
  }
  
  if (millis() - start_time >= SENSOR_DELAY_TIME){

    float humidity = dht.readHumidity();
    float temperature_celsius = dht.readTemperature();
    float temperature_fahrenheit = dht.readTemperature(true);
    float hic = dht.computeHeatIndex(temperature_fahrenheit, humidity, false);
    float hif = dht.computeHeatIndex(temperature_fahrenheit, humidity);
    // int speed_based_on_humidity = 0;
    // int speed_based_on_temperature = 0;
    if (humidity > HUMIDITY_THRESHOLD || temperature_celsius > TEMPERATURE_THRESHOLD){
      digitalWrite(DC_MOTOR_PIN, HIGH);
    }
    else {
      digitalWrite(DC_MOTOR_PIN, LOW);
    }
    
    // // Evaluate fan speed needed based on humidity
    // if (humidity >= HUMIDITY_THRESHOLD[0] && humidity < HUMIDITY_THRESHOLD[1]){
    //   speed_based_on_humidity = 1;
    // }
    // else if (humidity >= HUMIDITY_THRESHOLD[1] && humidity < HUMIDITY_THRESHOLD[2]){
    //   speed_based_on_humidity = 2;
    // }
    // else if (humidity > HUMIDITY_THRESHOLD[2]){
    //   speed_based_on_humidity = 3;
    // }

    // // Evaluate fan speed needed based on temperature
    // if (temperature_celsius >= TEMPERATURE_THRESHOLD[0] && temperature_celsius < TEMPERATURE_THRESHOLD[1]){
    //   speed_based_on_temperature = 1;
    // }
    // else if (temperature_celsius >= TEMPERATURE_THRESHOLD[1] && temperature_celsius < TEMPERATURE_THRESHOLD[2]){
    //   speed_based_on_temperature = 2;
    // }
    // else if (temperature_celsius > TEMPERATURE_THRESHOLD[2]){
    //   speed_based_on_temperature = 3;
    // }

    // int final_speed = max(speed_based_on_humidity, speed_based_on_temperature);
    // if (speed_based_on_humidity != 0 && speed_based_on_temperature != 0)
    // {
    //   analogWrite(DC_MOTOR_PIN, DC_MOTOR_FAN_SPEED[final_speed]);
    // }

    char payload[1000];
    sprintf(
      payload,
      "Humidity (%%): %.2f, Temperature (C): %.2f, Temperature (F): %.2f, Heat Index (C): %.2f, Heat Index (F): %.2f",
      humidity, temperature_celsius, temperature_fahrenheit, hic, hif);
    client.publish(MQTT_TOPIC_COOLING, payload);
    is_start_time_set = false;
    is_dht_begin = false;
    return true;
  }

  return false;
}

bool hiveShadingFunction(){
  static bool is_servo_rotation_needed = false;
  static bool is_shade_open = false;
  static bool is_start_time_set = false;
  static unsigned long read_start_time = millis();
  static int stop_angle = 180;
  static bool is_reading_read = false;
  int rain_intensity;
  int light_intensity;

  if (!is_reading_read) {
    if (!is_start_time_set) {
      read_start_time = millis();
      is_start_time_set = true;
    }

    if (millis() - read_start_time >= SENSOR_DELAY_TIME){
      rain_intensity = map(analogRead(RAIN_SENSOR_ANALOG_PIN), 4095, 0, 0, 100);
      light_intensity = map(analogRead(LDR_SENSOR_ANALOG_PIN), 4095, 0, 0, 100);
      char payload[1000];
      sprintf(payload, "Rain: %s, Rain Intensity (%%): %d, Light Intensity (%%): %d", rain_intensity >= RAIN_INTENSITY_THRESHOLD ? "True" : "False", rain_intensity, light_intensity);
      client.publish(MQTT_TOPIC_SHADE, payload);
      is_reading_read = true;
    }
  }

  if (is_reading_read && !is_servo_rotation_needed){
    // It is raining but the shade is closed (Open shade)
    if ((rain_intensity >= RAIN_INTENSITY_THRESHOLD || light_intensity >= LIGHT_INTENSITY_THRESHOLD) && !is_shade_open) {
      is_servo_rotation_needed = true;
      stop_angle = 180;
      is_shade_open = true;
    }
    // It is not raining but the shade is opened (Close shade)
    else if ((rain_intensity < RAIN_INTENSITY_THRESHOLD && light_intensity < LIGHT_INTENSITY_THRESHOLD) && is_shade_open) {
      is_servo_rotation_needed = true;
      stop_angle = 0;
      is_shade_open = false;
    }
    // No action needed
    else {
      is_servo_rotation_needed = false;
      is_reading_read = false;
      is_start_time_set = false;
      return true;
    }
  }

  if (is_reading_read && is_servo_rotation_needed) {
    myServo.write(stop_angle);
    is_reading_read = false;
    is_start_time_set = false;
    is_servo_rotation_needed = false;
    return true;
  }
  return false;
}

void loop() {
  static unsigned long last_execution_time = 0;
  static bool is_vperipheral_enabled = false;
  static bool is_cooling_executed = false;
  static bool is_shading_executed = false;
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Always on function
  ingressEgressCounterFunction();

  // Scheduled functions
  if (millis() - last_execution_time > VPERIPHERAL_DELAY_TIME){
    // Switch on vperipheral if haven't
    if (!is_vperipheral_enabled){
      digitalWrite(CONTROL_PIN, HIGH);
      is_vperipheral_enabled = true;
    }
    /**
    The scheduled tasks will be executed based on their execution statuses
    - False: in progress
    - True: completed
    **/
    if (!is_cooling_executed) is_cooling_executed = hiveCoolingFunction();
    if (!is_shading_executed) is_shading_executed = hiveShadingFunction();
  }

  if (is_cooling_executed && is_shading_executed) {
    // Switch off vperipheral
    digitalWrite(CONTROL_PIN, LOW);
    last_execution_time = millis();
    is_vperipheral_enabled = false;
    is_cooling_executed = false;
    is_shading_executed = false;
  }
}