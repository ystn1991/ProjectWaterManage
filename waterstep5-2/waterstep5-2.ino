#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <NewPing.h>

#define TRIGGER_PIN_1 12
#define ECHO_PIN_1 13
#define TRIGGER_PIN_2 32
#define ECHO_PIN_2 33
#define RELAY_PIN 27
#define RELAY_PIN_2 26

#define MQTT_SERVER "broker.emqx.io"
#define MQTT_PORT 1883

const char* ssid = "TrueGigatexFiber_230";
const char* password = "0922612764";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
HTTPClient httpClient;

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2);

const float TANK_CAPACITY = 2.0;

const char* MQTT_TOPIC_TANK1 = "tank1";
const char* MQTT_TOPIC_TANK2 = "tank2";
const char* MQTT_TOPIC_CONTROL = "tank1/control";
const char* MQTT_TOPIC_STATUS = "tank1/status";
const char* MQTT_TOPIC_TANK2_STATUS = "tank2/status";

bool relayStatus = false;
bool tank2RelayStatus = false;
bool fillingWater = false;

float waterLevel1_ml = 0.0;
float waterLevel2_ml = 0.0;
float waterNeeded_ml = 0.0;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected");
      mqttClient.subscribe(MQTT_TOPIC_CONTROL);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(1000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  msg[length] = '\0';
  Serial.println(msg);

  if (String(topic) == MQTT_TOPIC_CONTROL) {
    waterNeeded_ml = atof(msg);
    fillingWater = true;
    digitalWrite(RELAY_PIN, LOW);
    relayStatus = true;
    mqttClient.publish(MQTT_TOPIC_STATUS, "1");
  }
}

void setup() {
  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  Serial.begin(115200);
  setup_wifi();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callback);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect_mqtt();
  }
  mqttClient.loop();

  unsigned long now = millis();
  static unsigned long lastMsg = 0;
  if (now - lastMsg > 2000) {
    lastMsg = now;
    
    int distance1_cm = sonar1.ping_cm();
    waterLevel1_ml = TANK_CAPACITY * (1 - (distance1_cm / 19.0));
    
    int distance2_cm = sonar2.ping_cm();
    waterLevel2_ml = TANK_CAPACITY * (1 - (distance2_cm / 19.0));

    if (fillingWater && waterLevel1_ml < waterNeeded_ml) {
      digitalWrite(RELAY_PIN, LOW);
      relayStatus = true;
    } else {
      digitalWrite(RELAY_PIN, HIGH);
      relayStatus = false;
      fillingWater = false;
      waterNeeded_ml = 0.0;
    }

    if (waterLevel2_ml < (0.8 * TANK_CAPACITY) && waterLevel2_ml < (0.9 * TANK_CAPACITY)) {
      digitalWrite(RELAY_PIN_2, LOW);
      tank2RelayStatus = true;
    } else {
      digitalWrite(RELAY_PIN_2, HIGH);
      tank2RelayStatus = false;
    }

    mqttClient.publish(MQTT_TOPIC_TANK1, String(waterLevel1_ml).c_str());
    mqttClient.publish(MQTT_TOPIC_TANK2, String(waterLevel2_ml).c_str());
    mqttClient.publish("waterNeeded", String(waterNeeded_ml).c_str());
    mqttClient.publish("maxCapacity", String(TANK_CAPACITY).c_str());
    mqttClient.publish(MQTT_TOPIC_STATUS, relayStatus ? "1" : "0");
    mqttClient.publish(MQTT_TOPIC_TANK2_STATUS, tank2RelayStatus ? "1" : "0");

    Serial.print("Tank 1 Water Level (ml): ");
    Serial.println(waterLevel1_ml);
    Serial.print("Tank 2 Water Level (ml): ");
    Serial.println(waterLevel2_ml);
    Serial.print("Water Needed (ml): ");
    Serial.println(waterNeeded_ml);
    Serial.print("Max Water Capacity (ml): ");
    Serial.println(TANK_CAPACITY);
    Serial.print("Relay Status: ");
    Serial.println(relayStatus ? "ON" : "OFF");
    Serial.print("Tank 2 Relay Status: ");
    Serial.println(tank2RelayStatus ? "ON" : "OFF");
  }
}
