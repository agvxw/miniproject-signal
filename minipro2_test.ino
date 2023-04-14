#include "mqtt_secrets (2).h"
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <TridentTD_LineNotify.h>
#include "sps30.h"


#define I2C_SDA 4
#define I2C_SCL 5
#define I2C_ADDR 0x4D

#define SP30_COMMS SERIALPORT1
#define TX_PIN 18
#define RX_PIN 19
#define DEBUG 0

#define LINE_TOKEN "ZdOhPxxgGLAHwmImh2I3J5A7gmDyzb8GKKi580Wxxxx"

int sw1 = 16;
int sw2 = 14;
int n = 0;


// replace with your wifi ssid and wpa2 key
const char *ssid = "iPhone";
const char *pass = "zzzzzzzz";
const char* server = "mqtt3.thingspeak.com";
const char* channelID = "20980xx";
const char* mqttUserName = SECRET_MQTT_USERNAME;
const char* mqttPass = SECRET_MQTT_PASSWORD;
const char* clientID = SECRET_MQTT_CLIENT_ID;

void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();


// create constructor
SPS30 sps30;



WiFiClient client;
PubSubClient mqtt(client);

Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();
float Temp_value ;
float read_temperature_LM73(void) {
  byte Temp_data[2];
  byte Temp_data_maske[2];
  float Temp_value;

  // I2C start and write slave address (0x4D)
  Wire1.beginTransmission(I2C_ADDR);
  Wire1.write(0); // write temperature address
  Wire1.endTransmission();

  // I2C start andread temperature data two byte
  Wire1.requestFrom(0x4D, 2 );
  Temp_data[1] = Wire1.read(); // receive MSB temp data
  Temp_data[0] = Wire1.read(); // receive LSB temp data

  // check Temperature under 0 degree
  if ((Temp_data[1] & 0x80) == 1) {
    Temp_data_maske[1] = ~Temp_data[1]; // 2'complement
    Temp_data_maske[0] = (~Temp_data[0] + 1) >> 5;
    Temp_value = ((Temp_data_maske[1] * 2) + (Temp_data_maske[0] *
    0.25)) * -1; //Temperature value in C degree
  }
  else {
    Temp_data_maske[1] = Temp_data[1] & 0b01111111;
    Temp_data_maske[0] = Temp_data[0] & 0b11100000;
    Temp_data_maske[0] = Temp_data[0] >> 5;
    Temp_value = (Temp_data_maske[1] * 2) + (Temp_data_maske[0] *0.25); //Temperature value in C degree
  }

/*  // print MSB temp data in HEX
  Serial.print("Temp_data[1](MSB) :");
  Serial.println(Temp_data[1], HEX);

  // print LSB temp data in HEX
  Serial.print("Temp_data[0](MSB) :");
  Serial.println(Temp_data[0], HEX);
  Serial.println("////////////////////////////////////////");*/

  //Temperature value in C degree
  Serial.print("Temp in C :");
  Serial.println(Temp_value);
  Serial.println("...........................");
  return Temp_value;
}

void show_temperature(int temp) {
  matrix.setTextColor(LED_ON);
  matrix.setTextSize(1);
  matrix.setRotation(1);
  matrix.setTextWrap(false);
  matrix.clear();
  matrix.setCursor(2, 1);
  matrix.print(String(temp));
  matrix.writeDisplay();
  return;
}
void show_pm(int pm) {
  matrix.setTextColor(LED_ON);
  matrix.setTextSize(1);
  matrix.setRotation(1);
  matrix.setTextWrap(false);
  matrix.clear();
  matrix.setCursor(2, 1);
  matrix.print(String(pm));
  matrix.writeDisplay();
  return;
}


void setup() {
  pinMode(sw1,INPUT_PULLUP);
  pinMode(sw2,INPUT_PULLUP);

  matrix.begin(0x70); // I2C start and write slave address (0x4D)
  Wire1.begin(I2C_SDA, I2C_SCL);
  Serial.begin (115200);
  mqtt.setServer(server, 1883);
  LINE.setToken(LINE_TOKEN);

  //serialTrigger((char *) "SPS30-Example1: Basic reading. press <enter> to start");

  Serial.println(F("Trying to connect"));

  // set driver debug level
  sps30.EnableDebugging(DEBUG);

  // set pins to use for softserial and Serial1 on ESP32
  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN,TX_PIN);

  // Begin communication channel;
  if (! sps30.begin(SP30_COMMS))
    Errorloop((char *) "could not initialize communication channel.", 0);

  // check for SPS30 connection
  if (! sps30.probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);
  else  Serial.println(F("Detected SPS30."));

  // reset SPS30 connection
  if (! sps30.reset()) Errorloop((char *) "could not reset.", 0);

  // read device info
  GetDeviceInfo();

  // start measurement
  if (sps30.start()) Serial.println(F("Measurement started"));
  else Errorloop((char *) "Could NOT start measurement", 0);

  //serialTrigger((char *) "Hit <enter> to continue reading");

  if (SP30_COMMS == I2C_COMMS) {
    if (sps30.I2C_expect() == 4)
      Serial.println(F(" !!! Due to I2C buffersize only the SPS30 MASS concentration is available !!! \n"));
  }
}

void loop() {
  int sw1Val = digitalRead(sw1);
  int sw2Val = digitalRead(sw2);
  
  unsigned static int half_second_count = 0; //

  float temp = read_temperature_LM73();
  int tempCorse = round(temp);
  
  uint8_t ret, error_cnt = 0;
  struct sps_values val;
  ret = sps30.GetValues(&val);
  
  float pm = val.MassPM2;
  int pmCorse = round(pm);
  Serial.print("pm2.5 in C :");
  Serial.println(pm);
  Serial.println("...........................");
  if(n == 0){
    show_temperature(tempCorse);


    n=1;
    
    }
  else if(sw2Val==0 && n == 1) {
    show_pm(pmCorse);


    n=2;
    
    }
  else if(sw1Val==0 && n == 2) {
    n=0;
    }
  
  // Check if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    //Connect to Router
    Serial.print("Connecting to :");
    Serial.print(ssid);
    Serial.print(" : ");
    //Connect ESP32 to home network
    WiFi.begin(ssid, pass);
    //Wait until Connection is complete
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nConnected.");
  }
  //Check if MQTT Server is connected
  if (!mqtt.connected()) {
    Serial.println("Connecting to MQTT Broker.");
    mqtt.connect(clientID, mqttUserName, mqttPass);
    while (!mqtt.connected()) {
      delay(500);
      Serial.print(".");
    }
  }
  //Post http get to ThingSpeak Every 15 seconds
  if (half_second_count >= 30) {
     
     //SendLINE
     if(temp<35|| pm < 25){
      LINE.notify("TEMP = "+String(temp,2)+"c\n"+"PM2.5 = "+String(pm)+"μg/m3");
      Serial.println("SEND LINE");
      
    }
    //Reset Timer
    half_second_count = 0;
    
    String dataString = "&field1=" + String(temp);
    String dataString2 = "&field2=" + String(pm);
    String topicString = "channels/" + String(channelID)+ "/publish";
    mqtt.publish( topicString.c_str(), dataString.c_str());
    mqtt.publish( topicString.c_str(), dataString2.c_str());
    Serial.println("temp = " + String(temp));
    Serial.println("pm2.5 = " + String(pm));
    Serial.println("MQTT UPDATE");

  }



  
  //Update Half seconds every 500 ms
  delay(500);
  
  half_second_count++;
}

void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK) {
    Serial.print(F("Serial number : "));
    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK)  {
    Serial.print(F("Product name  : "));

    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK) {
    Serial.println(F("Can not read version info"));
    return;
  }

  Serial.print(F("Firmware level: "));  Serial.print(v.major);
  Serial.print("."); Serial.println(v.minor);

  if (SP30_COMMS != I2C_COMMS) {
    Serial.print(F("Hardware level: ")); Serial.println(v.HW_version);

    Serial.print(F("SHDLC protocol: ")); Serial.print(v.SHDLC_major);
    Serial.print("."); Serial.println(v.SHDLC_minor);
  }

  Serial.print(F("Library level : "));  Serial.print(v.DRV_major);
  Serial.print(".");  Serial.println(v.DRV_minor);
}



/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 *  @param r : error code
 *
 *  if r is zero, it will only display the message
 */
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

/**
 * serialTrigger prints repeated message, then waits for enter
 * to come in from the serial port.
 */
void serialTrigger(char * mess)
{
  Serial.println();

  while (!Serial.available()) {
    Serial.println(mess);
    delay(2000);
  }

  while (Serial.available())
    Serial.read();
}
