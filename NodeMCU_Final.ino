#include<SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <FirebaseArduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define FIREBASE_HOST "power-event-detector-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "ArW7BPjIqww3goBkb85L6rtDwNaxuyemDk7oc1j2"
#define WIFI_SSID "akpati1"
#define WIFI_PASSWORD "akshaya11"

const long utcOffsetInSeconds = 19800;

int hh,mm,ss;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

SoftwareSerial NodeMcu_SoftSerial(D3,D4); // RX,TX

//Below is Global Variable Data

char c; 
String DataIn;
int8_t indexofA, indexofB, indexofC;
String V_RMS,Frequency,Current;

void setup() {

  //Open Serial Communication (NodeMcu -PC)

  Serial.begin(57600);

  lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.backlight();

  //Open Serial communication (NodeMcu -Arduino)

  NodeMcu_SoftSerial.begin(9600);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected: ");
  Serial.println(WiFi.localIP());
  
  Firebase.begin(FIREBASE_HOST,FIREBASE_AUTH);
  timeClient.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  timeClient.update();
  hh=timeClient.getHours();
  mm=timeClient.getMinutes();
  ss=timeClient.getSeconds();
  String h=String(hh);
  String m=String(mm);
  String s=String(ss);
  String i=":";
  String t=h+i+m+i+s;
  while(NodeMcu_SoftSerial.available()>0) {
    
    c= NodeMcu_SoftSerial.read();

    
    if(c== '\n') {break;} 
    else {DataIn+=c;}
  }

  
    if(c== '\n')
    {
        Parse_the_data();


        //Show All data to the Serial Monitor

          Serial.println("RMS Voltage = " + V_RMS);
          Serial.println("Frequency = " + Frequency);
          Serial.println("Current = " + Current);
          Serial.println("====================================");

          float rmsvoltage=V_RMS.toFloat();
          float frequency=Frequency.toFloat();
          float current=Current.toFloat();
          
          Firebase.setFloat("RMS Voltage",rmsvoltage);
          Firebase.setFloat("RMS Current",current);
          Firebase.setFloat("Frequency",frequency);
          
          if(frequency<49)
          {
            Firebase.pushString("Warning","Frequency Fluctuation Detected at "+t);
            lcd.setCursor(0,0);
            lcd.print("Frequency Fluctuation dete");
            lcd.setCursor(0,1);
            lcd.print("cted at "+t);
          }
          if(frequency>51)
          {
            Firebase.pushString("Warning","Frequency Fluctuation Detected at "+t);
            lcd.setCursor(0,0);
            lcd.print("Frequency Fluctuation dete");
            lcd.setCursor(0,1);
            lcd.print("cted at "+t);
          }
          if(rmsvoltage<22)
          {
            Firebase.pushString("Warning","Volatge Sag Detected at "+t);
            lcd.setCursor(0,0);
            lcd.print("Voltage Sag dete");
            lcd.setCursor(0,1);
            lcd.print("cted at "+t);
          }
          if(rmsvoltage>28)
          {
            Firebase.pushString("Warning","Voltage Swell Detected at "+t);
            lcd.setCursor(0,0);
            lcd.print("Voltage Sag dete");
            lcd.setCursor(0,1);
            lcd.print("cted at "+t);
          }
  
          if (Firebase.failed()){
          Serial.println("setting /number failed:");
          Serial.println(Firebase.error());  
          return;
          }

        //Reset the variable

        c=0;
        DataIn="";  

    }
}

/////////////////////////////////////////////////////////////////////////////////////////////


void Parse_the_data()
{
  indexofA = DataIn.indexOf("A");
  indexofB = DataIn.indexOf("B");
  indexofC = DataIn.indexOf("C");

  V_RMS  = DataIn.substring (0, indexofA);
  Current = DataIn.substring (indexofA+1,indexofB);
  Frequency = DataIn.substring (indexofB+1,indexofC);
}
