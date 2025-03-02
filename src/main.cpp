#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include "PS2X_lib.h"

using namespace websockets;

/* WiFi configuration */
const char* ssid = "<YOUR SSID>";
const char* password = "<YOUR PASSWORD";
const char* rosbridge_server = "<YOUR ROSBRIDGE SERVER IP>";
const uint16_t rosbridge_port = 9090;
String ws_url = String("ws://") + rosbridge_server + ":" + String(rosbridge_port);

WebsocketsClient client;

// Callback to handle incoming messages (if needed)
void onMessageCallback(WebsocketsMessage message) {
  Serial.print("Received: ");
  Serial.println(message.data());
}

/**********************************************************/

// These are GPIO numbers, not pin numbers
#define PS2_DAT        12   
#define PS2_CMD        13 
#define PS2_SEL        0 
#define PS2_CLK        14 

PS2X ps2x;
byte vibrate = 0;

void advertiseJoyTopic() {
  // Construct the JSON document to advertise the /joy topic
  StaticJsonDocument<128> doc;
  doc["op"] = "advertise";
  doc["topic"] = "/joy";
  doc["type"] = "sensor_msgs/msg/Joy";

  String output;
  serializeJson(doc, output);
  
  client.send(output);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Credit: https://github.com/madsci1016/Arduino-PS2X/blob/master/PS2X_lib/examples/PS2X_Example/PS2X_Example.ino

  // Setup PS2 controller:
  uint8_t error=1, pressures=1,rumble=0;
 
  Serial.println("Connecting to PS2 controller...");
  while(error==1){
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
 
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
 
  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
   
  } // End while
  Serial.println("DONE");
 
  uint8_t type = ps2x.readType();
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }

  // SETUP WIFI
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  // Setup WebSocket callbacks and connect to rosbridge
  client.onMessage(onMessageCallback);
  Serial.print("Connecting to rosbridge at ");
  Serial.println(ws_url);
  if (client.connect(ws_url)) {
    Serial.println("Connected to rosbridge!");
    advertiseJoyTopic();  // Advertise the /joy topic
  } else {
    Serial.println("Connection failed!");
  }

}

void loop() {

  client.poll();

  int x1, y1, z1;
  int x2, y2, z2;

  ps2x.read_gamepad(false, vibrate);

  if (ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_RIGHT) || ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_DOWN)) {
    /*
    * Gamepad-based control. 
    */
    Serial.print("gamepad: ");
    Serial.println(ps2x.Button(PSB_PAD_LEFT));
    if (ps2x.Button(PSB_PAD_RIGHT)) { // if you push right, you want to go right
      y1 = 15; // so you drive forward with the left motor
      y2 = 245;
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      y1 = 245;
      y2 = 15;
    }

    // *backwards*, then *forwards* takes precedence in mutli-button presses
    if (ps2x.Button(PSB_PAD_DOWN)) {
      y1 = y2 = 245;
    }   
    if (ps2x.Button(PSB_PAD_UP)) {
      y1 = y2 = 15;
    }
  } else {
    
    //Joysticks
    Serial.print("joystick: ");
    x1 = ps2x.Analog(PSS_LX);
    y1 = ps2x.Analog(PSS_LY);
    z1 = ps2x.Button(PSB_L3);

    x2 = ps2x.Analog(PSS_RX);
    y2 = ps2x.Analog(PSS_RY);
    z2 = ps2x.Button(PSB_R3);
  }

  // some serial debugging output
  Serial.print(x1, DEC);
  Serial.print(",");
  Serial.print(y1, DEC);
  Serial.print("|");
  Serial.print(z1, DEC);
  Serial.println(" ");

  Serial.print(x2, DEC);
  Serial.print(",");
  Serial.print(y2, DEC);
  Serial.println(" ");
  Serial.print(z2, DEC);
  Serial.println(" ");

  // Construct the JSON document to publish to the /joy topic
  StaticJsonDocument<256> doc;
  doc["op"] = "publish";
  doc["topic"] = "/joy";

  // Build the message payload (mimicking sensor_msgs/Joy)
  JsonObject msg = doc.createNestedObject("msg");
  
  // Header with dummy timestamp (update if needed)
  JsonObject header = msg.createNestedObject("header");
  header["frame_id"] = "controller";
  JsonObject stamp = header.createNestedObject("stamp");
  stamp["sec"] = 0;
  stamp["nanosec"] = 0;
  
  // Axes array (e.g., joystick positions)
  JsonArray axes = msg.createNestedArray("axes");
  axes.add(x1);   // Left stick X
  axes.add(y1);   // Left stick Y
  axes.add(x2);   // Right stick X
  axes.add(y2);   // Right stick Y
  
  // Buttons array (digital button states: 0 or 1)
  JsonArray buttons = msg.createNestedArray("buttons");
  buttons.add(ps2x.Button(PSB_CROSS));    // Cross
  buttons.add(ps2x.Button(PSB_CIRCLE));   // Circle
  buttons.add(ps2x.Button(PSB_SQUARE));   // Square
  buttons.add(ps2x.Button(PSB_TRIANGLE)); // Triangle
  buttons.add(ps2x.Button(PSB_SELECT));   // Select
  buttons.add(ps2x.Button(PSB_START));    // Start
  buttons.add(ps2x.Button(PSB_L1));       // L1
  buttons.add(ps2x.Button(PSB_R1));       // R1
  buttons.add(ps2x.Button(PSB_L2));       // L2
  buttons.add(ps2x.Button(PSB_R2));       // R2
  buttons.add(ps2x.Button(PSB_L3));       // L3
  buttons.add(ps2x.Button(PSB_R3));       // R3

  // Serialize JSON document into a string
  String output;
  serializeJson(doc, output);
  
  // Send the JSON message over the WebSocket
  client.send(output);

  delay(20);
}