#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <NewPing.h>

#define I2C_SLAVE_ADDR 0x04
#define TRIGGER_PIN  19
#define ECHO_PIN     33
#define MAX_DISTANCE 100

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
WiFiClient espClient;
PubSubClient client(espClient);
MPU6050 mpu6050(Wire);

const char* ssid = "table13";    
const char* password = "benchthirteen";                            
const char* mqtt_server = "192.168.2.1";

long angle;
long distanceTravelled;
long encoderCount;
long lastMsg = 0;
int value = 0;
int steeringAngle;
int motorSpeed;
int ultrasonicDistance;
char msg[50];

void setup() 
{
  Serial.begin(115200);
  setup_wifi();
  Wire.begin();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
void setup_wifi() 
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void callback(char* topic, byte* message, unsigned int length) 
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "esp32/steeringangle")  //Subscribed Topics
  {
    steeringAngle = messageTemp.toInt();
    Serial.print("Changing Steering Angle to "); Serial.println(mpu6050.getAngleZ());
    sendMechanicalData(motorSpeed, motorSpeed, messageTemp.toInt());
  }

  if (String(topic) == "esp32/motorspeed")  //Subscribed Topics
  {
    motorSpeed = messageTemp.toInt();
    Serial.print("Changing Motor Speed to "); Serial.println(messageTemp);
    sendMechanicalData(messageTemp.toInt(), messageTemp.toInt(), steeringAngle);
  }
}
void reconnect() 
{

  while (!client.connected())  // Loop until we're reconnected
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client"))  // Attempt to connect
    {
      Serial.println("connected");
      
      client.subscribe("esp32/steeringangle");  //Subscribe Topics
      client.subscribe("esp32/motorspeed");   //Subscribe Topics
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}
void sendMechanicalData(int leftMotor, int rightMotor, int steeringAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor & 0x0000FF00) >> 8));
  Wire.write((byte)(leftMotor & 0x000000FF));
  Wire.write((byte)((rightMotor & 0x0000FF00) >> 8));
  Wire.write((byte)(rightMotor & 0x000000FF));
  Wire.write((byte)((steeringAngle & 0x0000FF00) >> 8));
  Wire.write((byte)(steeringAngle & 0x000000FF));
  Wire.endTransmission();
}
float readEncoderData ()
{
  Wire.requestFrom(I2C_SLAVE_ADDR, 4);

  int16_t enc1_count;
  int16_t enc2_count;
  uint8_t enc1_count16_9 = Wire.read();
  uint8_t enc1_count8_1 = Wire.read();
  uint8_t enc2_count16_9 = Wire.read();
  uint8_t enc2_count8_1 = Wire.read();

  enc1_count = (enc1_count16_9 << 8) | enc1_count8_1;
  enc2_count = (enc2_count16_9 << 8) | enc2_count8_1;

  long encoder = ((enc1_count + enc2_count) / 2);

  return(encoder);
}
long distanceCarTravelled ()
{
  long distance = (0.785398163397448 * readEncoderData());
  return (distance);
}
void loop()
{
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 10) 
  {
    lastMsg = now;

    mpu6050.update();
    
    angle = mpu6050.getAngleZ();
    distanceTravelled = distanceCarTravelled();
    ultrasonicDistance = sonar.ping_cm();
    encoderCount = readEncoderData();

    char angleString[8];
    dtostrf(angle, 1, 2, angleString);
    Serial.print("Angle = ");
    Serial.println(angleString);
    client.publish("esp32/angle", angleString);  //Publish Topics

    char distString[8];
    dtostrf(distanceTravelled, 1, 2, distString);
    Serial.print("Distance Travelled = ");
    Serial.println(distString);
    client.publish("esp32/distancetravelled", distString);  //Publish Topics

    char ultraString[8];
    dtostrf(ultrasonicDistance, 1, 2, ultraString);
    Serial.print("UltrasonicDistance = ");
    Serial.println(ultraString);
    client.publish("esp32/ultrasonicdistance", ultraString);  //Publish Topics

    char encString[8];
    dtostrf(encoderCount, 1, 2, encString);
    Serial.print("Encoder Count = ");
    Serial.println(encString);
    client.publish("esp32/encodercount", encString);  //Publish Topics
  }
}