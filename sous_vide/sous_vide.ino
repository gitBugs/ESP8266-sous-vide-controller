

//MQTT and WiFi Setup
//==============================================
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#undef MQTT_KEEPALIVE 
#define MQTT_KEEPALIVE 60



//char wifissid[] = "xxxx";   // Wifi network name or ESSID/SSID
//char wifipwd[] = "xxxx";   // Wifi password


char nodeprefix[] = "sousVideBot";   // Prefix for node name, rest will be mac address
char mqttsrvr[] = "192.168.1.80";    // Ip address of mqtt broker
//char mqttsrvr[] = "10.3.11.164";    // Ip address of mqtt broker

char mqttuid[] = "mqttusername";     // mqtt broker username
char mqttpwd[] = "mqrrpassword";     // mqtt broker password

char topic_node[] = "flatMQTT/nodes";   // topic name for LWT node up/down notifications
char topic_temp[] = "sousvide/temp";    // topic name for temperature readings
char topic_output[] = "sousvide/output";// topic name for output (during the next 10 seconds, for how many sec the heating element will be on)
String NodeName="";

// arrived message callback subroutine -- not used, hangover from old code
void callback(char* topic, byte* payload, unsigned int length) {
   // handle message arrived
   return;
   } 
   
//Initialize mqtt and wifi
WiFiClient wifiClient;
PubSubClient client(mqttsrvr, 1883, callback, wifiClient);

//Temperature sensor setup
//=================================================
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into GPIO14 on the ESP
#define ONE_WIRE_BUS 14

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//PID Setup
//=================================================
#include <PID_v1.h>
#define RelayPin 12  //Using GPIO13 to control the relay. 

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,8,15,5, DIRECT); //Fiddle with the PID variables to better adapt algo to the slow cooker. Currently, I and D need increasing, P probably steady or down.

int WindowSize = 10000; //length of the window/chunk of time we're considering, in ms. Longer = less power cycling of relay/slow cooker, probably a good thing. But also less fine control.
unsigned long windowStartTime;

//==================================================
void setup(){
 
  Serial.begin(115200);
  delay(500);
  Serial.println("Serial connected");
  WifiConnect();
  BrokerConnect(); 
  //Start temp sensor library
  sensors.begin();
  
  //PID settings
  pinMode(RelayPin, OUTPUT);
  windowStartTime = millis();
  
  //initialize the variables we're linked to
  Setpoint = 52; // This is the temperature that the sous vide will aim for, in C. Someday, add ability to receive this over MQTT or web form?

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

// Convert 6 byte mac address to text string subroutine
String macToStr(const uint8_t* mac)
{
   String result;
   
   for (int i = 0; i < 6; ++i) {
      result += String(mac[i], 16);
   }
   
   return result;
}
void WifiConnect() {
  int timout = 10;
  WiFi.mode(WIFI_STA);
  Serial.print("Wifi connecting to ");
  Serial.println(wifissid);
  WiFi.begin(wifissid, wifipwd);
  while (WiFi.status() != WL_CONNECTED) {
     delay(1000);
     Serial.print(".");
     if(--timout < 0){
        Serial.println("WIFI connect failed reset and try again...");
        abort();
     }       
  }
  Serial.println("");
  Serial.print("WiFi connected with IP ");
  Serial.println(WiFi.localIP()); 
  return;
}
// Connect to MQTT Broker subroutine -- NB: If this fails long-term (i.e. if laptop running broker turns off) then cooker will stop working! 
void BrokerConnect() {
   uint8_t mac[6];
   String lwtUp;
   String lwtDn; 
  
   if(NodeName.length() <= 0){
      // Generate client name and LWT messages based on MAC address and last 8 bits of microsecond counter
      WiFi.macAddress(mac);   
      NodeName = nodeprefix + macToStr(mac);
      lwtUp = NodeName + " is alive";
      lwtDn = NodeName + " is dead";
   } 
  
   Serial.print(NodeName);
   Serial.print(" connecting to ");
   Serial.print(mqttsrvr);
   Serial.print(" as ");
   Serial.println(mqttuid);

   if (client.connect((char*)NodeName.c_str(), mqttuid, mqttpwd, topic_node, 0, 0,  (char*)lwtDn.c_str())) {
      Serial.println("Connected to MQTT broker");
      Serial.print("LWT topic is: ");
      Serial.println(topic_node);
      if (client.publish(topic_node, (char*)lwtUp.c_str())) {
         Serial.println("Publish lwt up ok");
      } else {
         Serial.println("Publish lwt up failed");
      }
   } else {
      Serial.println("MQTT connect failed, reset and try again...");
   }

   return; 
}  


void loop()
{
  sensors.requestTemperatures(); //Send command to get temperature
  Input = sensors.getTempCByIndex(0); // Set PID input to temperatures;

  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
    Serial.print("Temperature = ");
    Serial.println(Input);
    Serial.print("Output = ");
    Serial.println(Output);
    char charInput[5];
    dtostrf(Input, 2, 3, charInput);
    client.publish(topic_temp, charInput);
    char charOutput[5];
    dtostrf(Output, 2, 3, charOutput);
    client.publish(topic_output, charOutput);
  }
  if(Output < millis() - windowStartTime) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);

  if (WiFi.status() != WL_CONNECTED){
    WifiConnect();
  }
  if (!client.connected()){
    BrokerConnect();
  }
}





