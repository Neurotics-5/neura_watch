#include <Arduino.h>
#include <micro_ros_platformio.h> 
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

// WiFi credentials
char ssid[] = "name";
char password[] = "password";
IPAddress agent_ip(000,000,0,000); 

// micro-ROS
rcl_publisher_t publisher;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__String msg;
char message[] = "Hello from ESP32";
unsigned long lastPub = 0;

// Pin definitions
const int motorPin = 14;
const int button1Pin = 26;
const int button2Pin = 32;
const int ledPin = 25;

// State tracking
bool lastB1 = false;
bool lastB2 = false;
unsigned long lastBlink = 0;
bool ledState = false;

// Vibration function
void vibrate(float intensity = 0.5, int duration_ms = 200) {
  int duty = intensity * 255;
  ledcWrite(0, duty);
  delay(duration_ms);
  ledcWrite(0, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  ledcSetup(0, 1000, 8);
  ledcAttachPin(motorPin, 0);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

set_microros_wifi_transports(ssid, password, agent_ip, 8888);

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp32_hello"
  );
}

void loop() {
  // Blink LED
  if (millis() - lastBlink >= 1000) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    lastBlink = millis();
    Serial.println("Loop running...");
  }

  // Buttons
  bool b1 = digitalRead(button1Pin) == LOW;
  bool b2 = digitalRead(button2Pin) == LOW;

  if (b1 && !lastB1) {
    Serial.println("Button 1 pressed");
    vibrate(0.8, 150);
  }
  if (b2 && !lastB2) {
    Serial.println("Button 2 pressed");
    vibrate(0.5, 100);
  }
  lastB1 = b1;
  lastB2 = b2;

  // Publish hello every 2 seconds
  if (millis() - lastPub >= 2000) {
    msg.data.data = message;
    msg.data.size = strlen(message);
    msg.data.capacity = sizeof(message);
    rcl_publish(&publisher, &msg, NULL);
    lastPub = millis();
  }

  delay(50);
}
