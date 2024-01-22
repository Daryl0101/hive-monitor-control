#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ESP32Servo.h>
#include "environ.h"

const int DHT_PIN = 42; //DA
const int DHT_TYPE = DHT11;
const float TEMPERATURE_THRESHOLD = 35;
const float HUMIDITY_THRESHOLD = 85;

const int IR_SENSOR_1_PIN = 14;
const int IR_SENSOR_2_PIN = 21;
const int PASS_FIRST_IR_MAX_TIME = 5000;
const int PASS_SECOND_IR_MAX_TIME = 5000;

const int RAIN_SENSOR_PIN = A5;
const float RAIN_INTENSITY_THRESHOLD = 1200;

const int DC_MOTOR_PIN = A0;
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
  // myServo.write(0);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  pinMode(11, OUTPUT);
  pinMode(DC_MOTOR_PIN, OUTPUT);
  pinMode(IR_SENSOR_1_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_2_PIN, INPUT_PULLUP);
}

void ingressEgressCounterFunction(){
  static const char *gress_types[3] = {"NA", "Ingress", "Egress"};
  static char *current_gress_type = gress_types[0];
  static unsigned long pass_first_ir_time = 0;
  static unsigned long pass_second_ir_time = 0;

  if (pass_second_ir_time > 0){
    if (millis() - pass_second_ir_time > PASS_SECOND_IR_MAX_TIME){
      current_gress_type = gress_types[0];
      pass_first_ir_time = 0;
      pass_second_ir_time = 0;
    }
  }
  else {
    if (current_gress_type == gress_types[0])
    {
      if (digitalRead(IR_SENSOR_1_PIN) == 0){
        current_gress_type = gress_types[1];
        pass_first_ir_time = millis();
      }
      else if (digitalRead(IR_SENSOR_2_PIN) == 0){
        current_gress_type = gress_types[2];
        pass_first_ir_time = millis();
      }
    }
    else if (current_gress_type == gress_types[1]) {
      if (millis() - pass_first_ir_time < PASS_FIRST_IR_MAX_TIME) {
        // check if another IR sensor is triggered
        if (digitalRead(IR_SENSOR_2_PIN) == 0){
          Serial.println("Ingress Detected!!");
          char payload[100];
          sprintf(payload, "Type: %s, Interval (s): %.2f", current_gress_type, float(pass_second_ir_time-pass_first_ir_time)/1000);
          client.publish(MQTT_TOPIC_MOVEMENT, payload);
          pass_second_ir_time = millis();
        }
      }
      else {
        current_gress_type = gress_types[0];
        pass_first_ir_time = 0;
      }
    }
    else {
      if (millis() - pass_first_ir_time < PASS_FIRST_IR_MAX_TIME) {
        // check if another IR sensor is triggered
        if (digitalRead(IR_SENSOR_1_PIN) == 0){
          Serial.println("Egress Detected!!");
          char payload[100];
          sprintf(payload, "Type: %s, Interval (s): %.2f", current_gress_type, float(pass_second_ir_time-pass_first_ir_time)/1000);
          client.publish(MQTT_TOPIC_MOVEMENT, payload);
          pass_second_ir_time = millis();
        }
      }
      else {
        current_gress_type = gress_types[0];
        pass_first_ir_time = 0;
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
  // Serial.println(millis() - start_time);
  
  if (millis() - start_time > 2000){

    float humidity = dht.readHumidity();
    float temperature_celsius = dht.readTemperature();
    float temperature_fahrenheit = dht.readTemperature(true);
    float hic = dht.computeHeatIndex(temperature_fahrenheit, humidity, false);
    float hif = dht.computeHeatIndex(temperature_fahrenheit, humidity);
    if (humidity > HUMIDITY_THRESHOLD || temperature_celsius > TEMPERATURE_THRESHOLD){
      digitalWrite(DC_MOTOR_PIN, HIGH);
    }
    else{
      digitalWrite(DC_MOTOR_PIN, LOW);
    }
    char payload[300];
    sprintf(
      payload,
      "Humidity: %.2f, Temperature (C): %.2f, Temperature (F): %.2f, Heat Index (C): %.2f, Heat Index (F): %.2f",
      humidity, temperature_celsius, temperature_fahrenheit, hic, hif);
    client.publish(MQTT_TOPIC_COOLING, payload);
    is_start_time_set = false;
    is_dht_begin = false;
    return true;
  }

  return false;
}

bool hiveShadingFunction(){
  static bool is_tested = false;
  static bool is_shade_open = false;
  static unsigned long move_start_time = millis();
  static int start_angle = 0;
  static int stop_angle = 180;
  static int count_func_access = 0;
  float rain_intensity;
  if (count_func_access == 0) {
    rain_intensity = analogRead(RAIN_SENSOR_PIN);
    char payload[50];
    sprintf(payload, "Rain Intensity: %.2f", rain_intensity);
    client.publish(MQTT_TOPIC_SHADE, payload);
    count_func_access++;
  }

  // myServo.write(180);
  // delay(1000);
  // myServo.write(0);
  // delay(1000);
  // delay(3000);

  if (!is_tested){
    Serial.println("This is !is_tested");
    if (rain_intensity < RAIN_INTENSITY_THRESHOLD && !is_shade_open) {
      Serial.println("This is 01");
      Serial.println("This is 01");
      Serial.println("");
      is_tested = true;
      start_angle = 0;
      stop_angle = 180;
      move_start_time = millis();
      is_shade_open = true;
    }
    else if (rain_intensity >= RAIN_INTENSITY_THRESHOLD && is_shade_open) {
      Serial.println("This is 01");
      Serial.println("rain_intensity");
      Serial.println("");
      is_tested = true;
      start_angle = 180;
      stop_angle = 0;
      move_start_time = millis();
      is_shade_open = false;
    }
    else {
      Serial.println("I was here!!!");
      Serial.println(rain_intensity < RAIN_INTENSITY_THRESHOLD);
      Serial.println(myServo.read());
      Serial.println();
      Serial.println(rain_intensity >= RAIN_INTENSITY_THRESHOLD);
      Serial.println(myServo.read());
      count_func_access = 0;
      return true;
    }
  }

  if (is_tested) {
    unsigned long progress = millis() - move_start_time;
    Serial.println(progress);
    if (progress <= SERVO_MOTOR_MOVING_TIME) {
      long angle = map(progress, 0, SERVO_MOTOR_MOVING_TIME, start_angle, stop_angle);
      myServo.write(angle); 
    }
    else {
      count_func_access = 0;
      is_tested = false;
      return true;
    }
  }
  return false;
}

void loop() {
  static unsigned long last_execution_time = 0;
  static bool is_cooling_executed = false;
  static bool is_shading_executed = false;
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Always on function
  ingressEgressCounterFunction();

  // Scheduled functions
  if (millis() - last_execution_time > 10 * 1000){
    digitalWrite(11, HIGH);
    if (!is_cooling_executed) is_cooling_executed = hiveCoolingFunction();
    if (!is_shading_executed) is_shading_executed = hiveShadingFunction();
  }

  if (is_cooling_executed && is_shading_executed) {
    digitalWrite(11, LOW);
    last_execution_time = millis();
    is_cooling_executed = false;
    is_shading_executed = false;
  }
}