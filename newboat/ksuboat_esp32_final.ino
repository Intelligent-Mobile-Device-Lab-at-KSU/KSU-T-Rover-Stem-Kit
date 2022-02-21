#include <WiFi.h>
#include <WiFiUDP.h>
#include <ESP32Servo.h>
Servo myservo;

int servoPin = 14;
int pos = 90;
String manualSteering = "-100";
const int switchPin = 25;

// WiFi network name and password:
const char * networkName = "Delta21";
const char * networkPswd = "Christian";

String hostname = "espressif"; //used to be ESP32-KSU-BOAT

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "1.1.1.1"; //this will get overwritten automatically
const int udpPort = 5000;

//Are we currently connected?
boolean connected = false;

const char* initmsg = "HELLO";

//The udp library class
WiFiUDP udp;
char packet[255];
char reply[] = "Packet received!";

void setup(){
  // Initilize hardware serial:
  Serial.begin(115200);
  
  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
  pinMode(switchPin,OUTPUT);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin);
  myservo.write(pos);
}

void loop(){
  //only send data when connected
  if(connected){
    // If packet received...
    int packetSize = udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet! Size: ");
      Serial.println(packetSize);
      int len = udp.read(packet, 255);
      if (len > 0)
      {
        packet[len] = '\0';
      }
      Serial.print("Packet received: ");
      Serial.println(packet);

      // To-Do
      // If packet == "HELLO"
      //      Then: send: "OK"
      // else: it is a turn command so convert message to float and call servo code.  

      if (strcmp(packet, initmsg) == 0)
      {
        //data will be sent to server
        uint8_t buffer[3] = "OK";
        //send HELLO to server
        Serial.print("Remote IP: ");
        Serial.println(udp.remoteIP()); // Hotspot IP address
        Serial.print("Remote Port: ");
        Serial.println(udp.remotePort()); // Hotspot port
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buffer, 3);
        udp.endPacket();
        //Wait for 1 second
        //delay(1000);
        //udp.parsePacket();
      } 

      else 
      {
        String s = String(packet);
        if(s==manualSteering){
          digitalWrite(switchPin,LOW);
          delay(1);
        }else{
          digitalWrite(switchPin,HIGH);
          delay(1);
          pos = map(s.toInt(), -15, 15, 75, 105);
          Serial.print("Setting Servo Angle To: ");
          Serial.println(pos);
          myservo.write(pos);
        }
      }
    }
  }
  //Wait for 1 second
  delay(100);
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Define hostname
  WiFi.setHostname(hostname.c_str()); //define hostname
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! Phone IP: ");
          Serial.println(WiFi.gatewayIP());  
          //udpAddress = WiFi.gatewayIP();
          Serial.print("ESP32 IP: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          Serial.print("ESP32 listening on UDP port: ");
          Serial.println(udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}
