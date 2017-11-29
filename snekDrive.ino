#include <Adafruit_PWMServoDriver.h>

/*
 Basic MQTT example

 This sketch demonstrates the basic capabilities of the library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 
*/

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>


#define MIN 370
#define MAX 740
    int rawPitch = 0;
    int rawRoll = 0;
    int rawYaw = 0;
    int rawThrottle = 0;
    
    double pitch       = 0.0;
    double roll        = 0.0;
    double yaw         = 0.0;
    double throttle    = 0.0;

    bool trigger    = false;
    bool thumb      = false;
    bool button3    = false;
    bool button4    = false;
    bool button5    = false;
    bool button6    = false;
    bool button7    = false;
    bool button8    = false;
    bool button9    = false;
    bool button10   = false;
    bool button11   = false;
    bool button12   = false;

    int hat         = -1;

struct LowPass {
  double smoothing = 0;
  double value = 0;
  double lowPass(double input) {
    value += (input - value) / smoothing;
    return value;
  }

  void reset() {
    value = 0;
  }
};

LowPass pitchLp;
LowPass yawLp;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(10, 0, 0, 3);
IPAddress server(10, 0, 0, 2);


void deserialize(byte* payload, unsigned int length) {
  sscanf((char *) payload, "%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i", &rawPitch, &rawRoll, &rawYaw, &rawThrottle, &trigger, &thumb, &button3, &button4, &button5, &button6, &button7, &button8, &button9, &button10, &button11, &button12, &hat);
  pitch = (double) rawPitch / 100.0;
  roll = (double) rawRoll / 100.0;
  yaw = (double) rawYaw / 100.0;
  throttle = (double) rawThrottle / 100.0;
}

void callback(char* topic, byte* payload, unsigned int length) {
  deserialize(payload, length);
  drive();
}

double normalize(double d) {
  if (d > 1) {
    return 1;
  }
  if (d < -1) {
    return -1;
  }
  return d;
}

double dMap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

EthernetClient ethClient;
PubSubClient client(ethClient);

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      client.subscribe("Joystick/data");
      break;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  pitchLp.smoothing = 6;
  yawLp.smoothing = 6;
  Serial.begin(9600);

  client.setServer(server, 1883);
  client.setCallback(callback);

  Ethernet.begin(mac, ip);
  // Allow the hardware to sort itself out

  delay(1500);
  pwm.begin();
  pwm.setPWMFreq(100);
}

void drive() {
  double translation, rotation = 0;
  double leftOut, rightOut = 0;
  long leftDrive, rightDrive = 0;

  translation = pitchLp.lowPass(-pitch);
  rotation = yawLp.lowPass(roll);
  if (trigger) {
    if (translation < 0.0) {
      rotation = -rotation;
    }
    if (rotation > 0.0) {
      leftOut = translation;
      rightOut = translation * (1.0 - abs(rotation));
    } else if (rotation < 0.0) {
      rightOut = translation;
      leftOut = translation * (1.0 - abs(rotation));
    } else {
      leftOut = translation;
      rightOut = translation;
    }
    leftOut = -leftOut;
    leftDrive = (long) dMap(leftOut, -1.0, 1.0, MIN, MAX);
    rightDrive = (long) dMap(rightOut, -1.0, 1.0, MIN, MAX);
  } else {
    leftDrive = 0;
    rightDrive = 0;
    pitchLp.reset();
    yawLp.reset();
  }
  for (int i = 0; i < 3; i++) {
    pwm.setPWM(i, 0, leftDrive);
  }
  for (int i = 3; i < 6; i++) {
    pwm.setPWM(i, 0, rightDrive);
  }
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
