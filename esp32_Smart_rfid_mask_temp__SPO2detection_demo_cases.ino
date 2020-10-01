

/*************************************************** 
  This is a library example for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1747 3V version
  ----> https://www.adafruit.com/products/1748 5V version

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SPI.h>
#include <MFRC522.h>


#include <WiFi.h>
//#include <HTTPClient.h>
//#include <WiFiManager.h>

#include <WiFiClientSecure.h>
#include "floatToString.h"
#include <Timer.h>
#include <HTTPClient.h>
#include <ESP32httpUpdate.h>
//#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//
 Timer t1;
 const String FirmwareVer={"1.0"}; 
#define URL_fw_Version "https://raw.githubusercontent.com/ankur072/blinkledOTA/master/version.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/ankur072/blinkledOTA/master/esp32_Smart_rfid_mask_temp__SPO2detection_demo_cases.bin"
HTTPClient http;
int flag=0;
//WiFiManager wifiManager;
String Config="";
int u=0;
//Timer t;

int i=0;
int deviceCount = 0;
String esid="AirtelIMT";
String epass = "4mirrortech";
float celsius;
String TEMP1;
char TEMP[25];
String line;
int y=0;

#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"
#include "Wire.h"
#include "OakOLED.h"
//#include "Adafruit_GFX.h"

#include "NewPing.h"

 
// Define Constants
#define REPORTING_PERIOD_MS     10000
 uint32_t tsLastReport = 0;

#define TRIGGER_PIN_1  2
#define ECHO_PIN_1     2
#define TRIGGER_PIN_2  15
#define ECHO_PIN_2     15
#define MAX_DISTANCE 400
 
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
 
// Define Variables
 

float duration1; // Stores First HC-SR04 pulse duration value
float duration2; // Stores Second HC-SR04 pulse duration value
float distance1; // Stores calculated distance in cm for First Sensor
float distance2; // Stores calculated distance in cm for Second Sensor
int iterations = 1;

OakOLED oled;
PulseOximeter pox;
//int inputPin = 2;               // choose the input pin (for PIR sensor)
//int pirState = LOW;             // we start, assuming no motion detected
int val = 0;   
float sum2 = 0;
float ultra = 0;
float HR = 0.0;
float SPO2 = 0.0;
float HR_SUM = 0.0;
float SPO2_SUM = 0.0;
int H=0; 
int H1=0;
float sum3 = 0.0;
float ultra2 = 0;
int count=0;
int count1=0;

float FINAL_HR_SUM=0.0;
float FINAL_SPO2_SUM=0.0;


/* Object named oled, of the class Adafruit_SSD1306 */

const int buzzer = 13;
const int ledRed = 12 ;
const int ledGreen = 14;
 
int H2=0;
#define RST_PIN         5          // Configurable, see typical pin layout above
#define SS_PIN          4          // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

MFRC522::MIFARE_Key key;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

float sum = 0.0;
uint8_t s = 0, m = 0, h = 0;
float t=0.0;

const unsigned char bitmap [] PROGMEM=
{
0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x18, 0x00, 0x0f, 0xe0, 0x7f, 0x00, 0x3f, 0xf9, 0xff, 0xc0,
0x7f, 0xf9, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xf0,
0xff, 0xf7, 0xff, 0xf0, 0xff, 0xe7, 0xff, 0xf0, 0xff, 0xe7, 0xff, 0xf0, 0x7f, 0xdb, 0xff, 0xe0,
0x7f, 0x9b, 0xff, 0xe0, 0x00, 0x3b, 0xc0, 0x00, 0x3f, 0xf9, 0x9f, 0xc0, 0x3f, 0xfd, 0xbf, 0xc0,
0x1f, 0xfd, 0xbf, 0x80, 0x0f, 0xfd, 0x7f, 0x00, 0x07, 0xfe, 0x7e, 0x00, 0x03, 0xfe, 0xfc, 0x00,
0x01, 0xff, 0xf8, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x7f, 0xe0, 0x00, 0x00, 0x3f, 0xc0, 0x00,
0x00, 0x0f, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void FirmwareUpdate()
{
  //digitalWrite(LED_BUILTIN, LOW);
  http.begin(URL_fw_Version,"70 94 DE DD E6 C4 69 48 3A 92 70 A1 48 56 78 2D 18 64 E0 B7");     // check version URL
  delay(100);
  int httpCode = http.GET();            // get data from version file
  delay(100);
  String payload;
  if (httpCode == HTTP_CODE_OK)         // if version received
  {
    payload = http.getString();  // save received version
    Serial.println(payload );
  }
  else
  {
    Serial.print("error in downloading version file:");
    Serial.println(httpCode);

  }



  
  http.end();
  if (httpCode == HTTP_CODE_OK)         // if version received
  {
  if(payload.equals(FirmwareVer) )
  {   
     Serial.println("Device already on latest firmware version"); 
  }
  else
  {
     Serial.println("New firmware detected");
     digitalWrite(ledGreen, HIGH);
     delay(2);
     digitalWrite(ledRed, LOW);
     delay(2);
      oled.clearDisplay();
           delay(10);
           oled.setTextSize(2);            
           oled.setCursor(0,0);
           oled.print("PleaseWait");
                      // oled.print(" ");

           oled.setTextSize(2);            
           oled.setCursor(0,30);
           oled.print("SystemUpdating");
                      // oled.print(" ");
           oled.setTextSize(2);
           oled.cp437(true);
                      // oled.write(167);
           oled.display(); 
     WiFiClient client;

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    digitalWrite(LED_BUILTIN, HIGH);


    t_httpUpdate_return ret = ESPhttpUpdate.update(URL_fw_Bin,"","70 94 DE DD E6 C4 69 48 3A 92 70 A1 48 56 78 2D 18 64 E0 B7");
    
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    } 
  }
 }  
}

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 30000;

 void repeatedCall(){
    unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis) >= interval) 
    {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      FirmwareUpdate();
    }
 }


void setup() {
  Serial.begin(115200);
  Serial.println("VESRION 1.0");
  delay(500);
 // wifiManager.setAPStaticIPConfig(IPAddress(192,168,4,6), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.mode(WIFI_STA);
  WiFi.begin(esid.c_str(), epass.c_str()); 
  delay(3000);
     
//  Wire.begin();
//  Wire.setClock(100000);
  delay(100);

 //delay(100);
// // pinMode(buzzer, FUNCTION_3); 
  pinMode(buzzer, OUTPUT);
 pinMode(ledRed, OUTPUT);
pinMode(ledGreen, OUTPUT);
//
//  
  digitalWrite(ledGreen, HIGH);
  delay(2);
  digitalWrite(ledRed, LOW);
  // pinMode(inputPin, INPUT); 
  digitalWrite(buzzer, LOW);
  Serial.println("Adafruit MLX90614 test");  
//  
  oled.begin(); 
  oled.clearDisplay();  /* Clear oled */
  oled.setTextSize(3);  /* Select font size of text. Increases with size of argument. */
  oled.setTextColor(1);  /* Color of text*/
  oled.clearDisplay();  /* Clear oled */
      oled.setCursor(6, 15);
//     // sprintf(time, "%02d:%02d:%02d", h, m, s);
//      //drawStr(20, 30, "Kushal");
      oled.setTextSize(5);
   oled.setCursor(0,0);
  oled.print("WAIT");
  oled.cp437(true);
  oled.display();  
  //delay(200);
 mlx.begin();
   SPI.begin();        // Init SPI bus
//// 
   mfrc522.PCD_Init(); // Init MFRC522 card
   for(int i=0; i<10; i++)
    {
      Serial.print(mlx.readObjectTempF()); Serial.println("*F");
     mlx.readObjectTempF();
   }
//    // Prepare the key (used both as key A and as key B)
    // using FFFFFFFFFFFFh which is the default at chip delivery from the factory
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }
////
    Serial.println(F("Scan a MIFARE Classic PICC to demonstrate read and write."));
    Serial.print(F("Using key (for A and B):"));
    dump_byte_array(key.keyByte, MFRC522::MF_KEY_SIZE);
    Serial.println();
////
    Serial.println(F("BEWARE: Data will be written to the PICC, in sector #1"));
    //Serial.println("else case");
//      
//  //digitalWrite(led, HIGH);

// pox.setOnBeatDetectedCallback(onBeatDetected);
  
    t1.every(1*1000,task);
    t1.every(20*1000,repeatedCall);
   }

   
void onBeatDetected(){}
void loop() {
 // pox.update();
  t1.update();
}    
    
          

    
 // Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); 
           
            // mlx.begin();
            
          

//         for(int i=0; i<1000; i++)
//          {
//             pox.update();
//          SPO2_SUM = SPO2_SUM+pox.getSpO2();
//          Serial.println(HR_SUM);
//          }
          
//          Serial.println(HR);
//          Serial.println(SPO2);
            // Serial.print(mlx.readObjectTempF()); Serial.println("*F");
              
               
             /*else {
               mfrc522.PICC_ReadCardSerial();
              // H2=1;
              Card_found();
              
              }*/




void task()
{
    if ( ! mfrc522.PICC_IsNewCardPresent())
            {
             else_case();
          
            } else {
               mfrc522.PICC_ReadCardSerial();
              Card_found();
              }
  }

  
int Card_found()
{          
        dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
         if(mfrc522.uid.uidByte>0x00 )
          { 
            
          // digitalWrite(led, LOW);
           oled.clearDisplay();
           delay(10);
           oled.setTextSize(3);            
           oled.setCursor(0,25);
           oled.print("WELCOME");
                      // oled.print(" ");
           oled.setTextSize(2);
           oled.cp437(true);
                      // oled.write(167);
           oled.display(); 
           delay(500);
             digitalWrite(buzzer, HIGH);
             delay(100);
             digitalWrite(buzzer, LOW);
             delay(100);
        // Serial.print("ultrasonic1...");
       // Serial.println(ultra2);
            
             do {
           oled.clearDisplay();
           delay(10);
           oled.setTextSize(2.5);            
           oled.setCursor(25,10);
           oled.print("Please");
           oled.setTextSize(2);            
           oled.setCursor(0,30);
           oled.print("ComeCloser");
                      // oled.print(" ");
           oled.setTextSize(1);
           oled.cp437(true);
                      // oled.write(167);
           oled.display(); 
            for(int k=0;k<1000; k++)
          {
            duration1 = sonar1.ping_median(iterations);
            distance1 = (duration1 / 2) * 0.0343;
            sum3 = sum3+distance1;
            delay(2);
           // Serial.println(distance1);
            distance1 = 0;
            duration1 = 0;
            }
            Serial.println();
          ultra2 = sum3/1000;
           count = count+1;
         // Serial.println(ultra2);
          if(count==10 && ultra2 >10)
          { 
            H=0;
            count=0;
            oled.clearDisplay();
            break;
            
          }
        //Serial.println(ultra2);
           if( ultra2 <=10)
           {
            H=1;
           }
           else{
         //  ultra2 = 0;
           sum3 = 0;
           }
//           for(int k=0;k<1000; k++)
//          {
//            sum3 = sum3+ultrasonic1.Ranging(CM);
//            }
//            ultra2 = sum3/1000;
           
           //break;
      }while(ultra2>10);
        if (H==1) {
                        H=0;
                        delay(100);
                         ultra2 = 0;
                         sum3=0;
                          digitalWrite(ledGreen, HIGH);
                       delay(2);
                       digitalWrite(ledRed, LOW);
                         oled.clearDisplay();  /* Clear oled */
                        oled.setCursor(6, 15);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(2.5);
                        oled.print("Check Temp");
                        oled.cp437(true);
                        oled.display();  
                     
                       
                       //digitalWrite(led, LOW);
                       digitalWrite(buzzer, HIGH);
                       delay(100);
                       digitalWrite(buzzer, LOW);
                       delay(100);
                // check if the input is HIGH
               // digitalWrite(buzzer, HIGH);  // turn LED ON
             
                    delay(200);
                  // We only want to print on the output change, not state
                   
                         for(int k=0; k<500; k++)
                    {
                      sum=sum+mlx.readObjectTempF();
                      //yield();
                      //delay(10);
                    //Serial.print(mlx.readObjectTempF()); Serial.println("*F");
                    //t = mlx.readObjectTempC();
                    }
                    t= sum/500;
/**********************TEMP**************************************/   
                    // mlx.begin();
                    //Serial.println(ultrasonic.Ranging(CM));
                    if(t>85.00)
                    {
                      Serial.println("POSITION CORRECT");
                      t=0.00;
                      sum=0.00;
                      for(int k=0; k<500; k++)
                    {
                      sum=sum+mlx.readObjectTempF();
                      //delay(10);
                    //Serial.print(mlx.readObjectTempF()); Serial.println("*F");
                    //t = mlx.readObjectTempC();
                    }
                    t= sum/500;
                    t = t+3;
                    Serial.print(t); Serial.println("*F");
                    
            
                    
                    digitalWrite(buzzer, HIGH);
                    delay(100);
                    digitalWrite(buzzer, LOW);
                    delay(100);
                    Serial.println();
                    //delay(500);
                    }
                    else {
                        oled.clearDisplay();  /* Clear oled */
                        oled.setCursor(20, 0);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(3);
                        oled.print("Stand");
                        oled.setCursor(20, 30);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(3);
                        oled.print("Again");
                        oled.cp437(true);
                        oled.display();  
                        delay(3000);
                        Serial.println("NOT IN CORRECT POSITION");
                        return 1;
                      }
                      delay(1000);
/**********************ULTRASONIC (MASK)**************************************/
                   // Serial.print("ultrasonic2...");
                    for(int k=0;k<1000; k++)
                      {
                        duration2 = sonar2.ping_median(iterations);
                        distance2 = (duration2 / 2) * 0.0343;
                        sum2 = sum2+distance2;
                        //Serial.println(distance2);
                         delay(2);
                        distance2 = 0;
                        duration2 = 0;
                        
                        
                        }
                        ultra = sum2/1000;
                        Serial.println(ultra);
                    if(ultra<2)
                        {
                          ultra = 0;
                          sum2 = 0;
                        oled.clearDisplay();  /* Clear oled */

                       // oled.print(t);
                       if(t>=89 && t <=99.5)
                       {
                             digitalWrite(buzzer, HIGH);
                             delay(100);
                             digitalWrite(buzzer, LOW);
                             delay(100);
                            oled.clearDisplay();  /* Clear oled */
                            oled.setCursor(10, 0);
                           // sprintf(time, "%02d:%02d:%02d", h, m, s);
                            //drawStr(20, 30, "Kushal");
                            oled.setTextSize(2);
                           // oled.setCursor(0,0);
                            oled.print("Body Temp");
                            oled.setTextSize(3);
                            oled.setCursor(20,30);
                            oled.print(t);
                           // oled.print(" ");
                            oled.setTextSize(1);
                            oled.cp437(true);
                            oled.write(167);
                            oled.setTextSize(2);
                            oled.print("F");
                            t = 0.0;
                            oled.display();
                            delay(3000); 
                        } 
                       else if(t >99.5)
                       {
                        if(t>100)
                        {
                              oled.clearDisplay();  /* Clear oled */
                            oled.setCursor(10, 0);
                           // sprintf(time, "%02d:%02d:%02d", h, m, s);
                            //drawStr(20, 30, "Kushal");
                            oled.setTextSize(2);
                           // oled.setCursor(0,0);
                            oled.print("Body Temp");
                            oled.setTextSize(3);
                           oled.setCursor(0,30);
                            oled.print(t);
                           // oled.print(" ");
                            oled.setTextSize(1);
                            oled.cp437(true);
                            oled.write(167);
                            oled.setTextSize(2);
                            oled.print("F");
                            oled.display(); 
                        }
                        else {
                                oled.clearDisplay();  /* Clear oled */
                              oled.setCursor(10, 0);
                             // sprintf(time, "%02d:%02d:%02d", h, m, s);
                              //drawStr(20, 30, "Kushal");
                              oled.setTextSize(2);
                             // oled.setCursor(0,0);
                              oled.print("Body Temp");
                              oled.setTextSize(3);
                              oled.setCursor(20,30);
                              oled.print(t);
                             // oled.print(" ");
                              oled.setTextSize(1);
                              oled.cp437(true);
                              oled.write(167);
                              oled.setTextSize(2);
                              oled.print("F");
                              oled.display(); 
                              t = 0.0;
                        }
                        //oled.oled(); 
                         t = 0.0;
                         delay(1000);
                         
                       oled.clearDisplay();  /* Clear oled */
                       delay(10);
                        oled.setCursor(10, 0);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(2);
                       // oled.setCursor(0,0);
                        oled.print("HIGH Temp");
            
                         oled.setTextSize(2);            
                        oled.setCursor(0,30);
                        oled.print("Access Denied");
                       // oled.print(" ");
                        oled.setTextSize(1);
                        oled.cp437(true);
                       // oled.write(167);
            
                        oled.display(); 
                         for(int u=0;u<10;u++)
                           {
                             digitalWrite(buzzer, HIGH);
                             delay(100);
                             digitalWrite(buzzer, LOW);
                             delay(100);
                           }                        
                              sum2 = 0;
                              ultra = 0;
                              t = 0;
                              sum = 0;
                              delay(1000);  
//                              digitalWrite(led, HIGH);
//                              delay(2);
//                              digitalWrite(led, LOW);
                              return 1;
                       }
                       else {
                          oled.clearDisplay();  /* Clear oled */
                        oled.setCursor(20, 0);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(3);
                        oled.print("Stand");
                        oled.setCursor(20, 30);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(3);
                        oled.print("Again");
                        oled.cp437(true);
                        oled.display();  
                        delay(3000);  
//                        digitalWrite(led, HIGH);
//                        delay(2);
//                        digitalWrite(led, LOW);
                        return 1;
                        }
                        //oled.oled();  
                          delay(1000);  
                          // Serial.println(ultra);
                           Serial.println("Mask Detected");
                             digitalWrite(buzzer, HIGH);
                             delay(100);
                             digitalWrite(buzzer, LOW);
                             delay(100);
                             sum2 = 0;
                            ultra = 0;
                            t = 0;
                            sum = 0;
                          }
                       else {
                          // Serial.println(ultra);
                           Serial.println("Mask NOT Detected");
                         oled.clearDisplay();  /* Clear oled */
                       delay(10);
                        oled.setTextSize(2);            
                        oled.setCursor(0,30);
                        oled.print("Access Denied");
                       // oled.print(" ");
                        oled.setTextSize(1);
                        oled.cp437(true);
                       // oled.write(167);
                        oled.display(); 
                        delay(500);
                         oled.clearDisplay();  /* Clear oled */
                       delay(10);
                         oled.setTextSize(2);            
                        oled.setCursor(0,30);
                        oled.print("Please wear the mask");
                       // oled.print(" ");
                        oled.setTextSize(1);
                        oled.cp437(true);
                       // oled.write(167);
                        oled.display(); 
                           for(int u=0;u<10;u++)
                           {
                             digitalWrite(buzzer, HIGH);
                             delay(100);
                             digitalWrite(buzzer, LOW);
                             delay(100);
                           } 
                              delay(1000);                    
                              sum2 = 0;
                              ultra = 0;
                              t = 0;
                              sum = 0;
//                              digitalWrite(led, HIGH);
//                              delay(2);
//                              digitalWrite(led, LOW);
                              return 1;
                        }
/**********************SPO2**************************************/

           
//          for(int i=0; i<1000; i++)
//          {
//          HR_SUM = HR_SUM+pox.getHeartRate();
//          }
//          HR = HR_SUM/1000;
//
//         for(int i=0; i<1000; i++)
//          {
//          SPO2_SUM = SPO2_SUM+pox.getSpO2();
//          }
//          SPO2 = SPO2_SUM/1000;

        // Please place finger with pause
        do {
           oled.clearDisplay();
           delay(10);
           oled.setTextSize(2);            
           oled.setCursor(20,0);
           oled.print("Please");
           oled.setTextSize(2);            
           oled.setCursor(0,30);
           oled.print("PlaceFinger");
                      // oled.print(" ");
           oled.setTextSize(1);
           oled.cp437(true);
                      // oled.write(167);
           oled.display(); 
//            for(int k=0;k<1000; k++)
//          {
//            sum3 = sum3+ultrasonic1.Ranging(CM);
//            }
//          ultra2 = sum3/1000;
          pox.begin();
         pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
        // delay(100);
         
          for(int i=0; i<100; i++)
          {
            pox.update();
            delay(100);
          HR_SUM = HR_SUM+pox.getHeartRate();
          SPO2_SUM = SPO2_SUM+pox.getSpO2();
          }
          HR = HR_SUM/100;
          SPO2 = SPO2_SUM/100;
           Serial.println("HR..");
            Serial.println(HR);
            Serial.println("SPO2..");
            Serial.println(SPO2);
          count1 = count1+1;
          //Serial.println(count);
          if(count1==20)
          { 
            if(HR<40 && SPO2<70)
            {
            H1=0;
            count1=0;
            oled.clearDisplay();
            oled.setTextSize(2);            
           oled.setCursor(20,0);
           oled.print("Please");
           oled.setTextSize(2);            
           oled.setCursor(0,30);
           oled.print("TryAgain");
                      // oled.print(" ");
           oled.setTextSize(1);
           oled.cp437(true);
                      // oled.write(167);
           oled.display();
           HR=0;
           SPO2=0;
           count1=0;
           delay(4000);
            break;
            
          }
          }
          // count = count+1;
          //Serial.println(count);
         /* if((HR>=50) && (SPO2>=80))
          { 
            H1=0;
            //count=0;
            display.clearDisplay();
            break;
            
          }*/
        //Serial.println(ultra2);
           if(HR>=40 && SPO2>=70)
           {
            H1=1;
           }
           else{
         //  ultra2 = 0;
          HR_SUM = 0.0;
          SPO2_SUM = 0.0;
           //SPO2 = 0.0;
           //HR = 0.0;
           }
//           for(int k=0;k<1000; k++)
//          {
//            sum3 = sum3+ultrasonic1.Ranging(CM);
//            }
//            ultra2 = sum3/1000;
           
           //break;
      }while((HR<40) && (SPO2<70));
      // finger placed 
      
          if(H1==1)
          {//correct finger
            H1=0;
            HR=0;
            SPO2=0;
            oled.clearDisplay();
           delay(10);
           oled.setTextSize(2);            
           oled.setCursor(20,0);
           oled.print("Checking");
           oled.setTextSize(2);            
           oled.setCursor(0,30);
           oled.print("BloodOxidation");
                      // oled.print(" ");
           oled.setTextSize(1);
           oled.cp437(true);
                      // oled.write(167);
           oled.display();
           delay(20);
         pox.begin();
         pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
         delay(100);
         
          for(int i=0; i<100; i++)
          {
            pox.update();
            delay(100);
          FINAL_HR_SUM = FINAL_HR_SUM+pox.getHeartRate();
          FINAL_SPO2_SUM = FINAL_SPO2_SUM+pox.getSpO2();
          digitalWrite(buzzer, HIGH);
           delay(5);
           digitalWrite(buzzer, LOW);
           delay(5);
          } 
           HR = FINAL_HR_SUM/100;
          SPO2 = FINAL_SPO2_SUM/100;
           Serial.println("FINAL_HR..");
            Serial.println(HR);
            Serial.println("FINAL_SPO2..");
            Serial.println(SPO2);
            //Now checking correct value
            if(SPO2>=94&&SPO2<=100)
            {
              if(HR>=60&&HR<=100)
              {
                           oled.clearDisplay();  /* Clear oled */
                           oled.setCursor(0, 0);
                             // sprintf(time, "%02d:%02d:%02d", h, m, s);
                              //drawStr(20, 30, "Kushal");
                              oled.setTextSize(2);
                             // oled.setCursor(0,0);
                              oled.print("HR");
                              oled.setTextSize(2);
                              oled.setCursor(40,0);
                              oled.print(HR);
                             // oled.print(" ");
                              oled.setCursor(0, 30);
                             // sprintf(time, "%02d:%02d:%02d", h, m, s);
                              //drawStr(20, 30, "Kushal");
                              oled.setTextSize(2);
                             // oled.setCursor(0,0);
                              oled.print("SPO2");
                              oled.setTextSize(2);
                              oled.setCursor(60,30);
                              oled.print(SPO2);
                              oled.setTextSize(1);
                              oled.cp437(true);
                              oled.write(167);
                             
                              oled.display(); 
                              delay(4000);
                              HR = 0.0;
                              SPO2 = 0.0;
                              HR_SUM = 0.0;
                              SPO2_SUM = 0.0;
                              FINAL_HR_SUM=0;
                              FINAL_SPO2_SUM=0;
                              
            }
            }
            else {// abnormal reading
                       oled.clearDisplay();  /* Clear oled */
                        oled.setCursor(20, 0);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(2);
                        oled.print("ABNORMAL");
                        oled.setCursor(20, 30);
                       // sprintf(time, "%02d:%02d:%02d", h, m, s);
                        //drawStr(20, 30, "Kushal");
                        oled.setTextSize(2);
                        oled.print("HR/SPO2");
                        oled.cp437(true);
                        oled.display();  
                       // delay(3000);
                        for(int u=0;u<10;u++)
                           {
                             digitalWrite(buzzer, HIGH);
                             delay(100);
                             digitalWrite(buzzer, LOW);
                             delay(100);
                           } 
                              delay(3000);                    
                              HR = 0.0;
                              SPO2 = 0.0;
                              HR_SUM = 0.0;
                              SPO2_SUM = 0.0;
                              FINAL_HR_SUM=0;
                              FINAL_SPO2_SUM=0;
//                              digitalWrite(led, HIGH);
//                              delay(2);
//                              digitalWrite(led, LOW);
                              return 1;
                        //Serial.println("Incorrect Finger");
                       // return 1;
            }
          }
          //SPO2 = pox.getSpO2();
/**********************OUTING LOOP**************************************/
                digitalWrite(buzzer, HIGH);
                delay(2000);
                digitalWrite(buzzer, LOW);
                delay(100);  
//                digitalWrite(led, HIGH);
//                delay(2);
//                digitalWrite(led, LOW);
            
                    sum2 = 0;
                    ultra = 0;
                    t = 0;
                    sum = 0;
                     ultra2 = 0;
                     sum3=0;
                      HR = 0.0;
                      SPO2 = 0.0;
                      HR_SUM = 0.0;
                      SPO2_SUM = 0.0;
                      FINAL_HR_SUM=0;
                      FINAL_SPO2_SUM=0;

        
      /*   if ( ! mfrc522.PICC_IsNewCardPresent())
              return;
      
          // Select one of the cards
          if ( ! mfrc522.PICC_ReadCardSerial())
              return;
      
          // Show some details of the PICC (that is: the tag/card)
          //Serial.print(F("Card UID:"));
          dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
          Serial.println();
           if(t>1 && mfrc522.uid.uidByte>0x00 )
          {
             digitalWrite(buzzer, HIGH);
             delay(100);
             digitalWrite(buzzer, LOW);
             char time[30];
            oled.clearoled();  /* Clear oled */
        /*    oled.setCursor(10, 0);
           // sprintf(time, "%02d:%02d:%02d", h, m, s);
            //drawStr(20, 30, "Kushal");
            oled.setTextSize(2);
           // oled.setCursor(0,0);
            oled.print("Body Temp");
            oled.setTextSize(3);
            if(t<100){
            oled.setCursor(20,30);
            oled.print(t);
           // oled.print(" ");
            oled.setTextSize(1);
            oled.cp437(true);
            oled.write(167);
            oled.setTextSize(2);
            oled.print("F");
            t = 0.0;
            } else {
            oled.setCursor(0,30);
            oled.print(t);
           // oled.print(" ");
            oled.setTextSize(1);
            oled.cp437(true);
            oled.write(167);
            oled.setTextSize(2);
            oled.print("F");
             t = 0.0;
              }
            oled.oled();  
           // delay(100);
           // drawStr(40, 30, time);
            //drawStr(20, 50, "4Mirrortech");
           // oled.oled();
          // oled.startscrollleft(0x00, 0x0F);
            delay(200);
            Serial.println();
            delay(500);
      }
      else{
             
             char time[30];
      oled.clearoled();  /* Clear oled */
     /* oled.setCursor(0, 30);
     // sprintf(time, "%02d:%02d:%02d", h, m, s);
      //drawStr(20, 30, "Kushal");
      oled.setTextSize(3);
     // oled.setCursor(0,0);
      oled.print("Please stand in correct position");
      oled.startscrollleft(0x00, 0x0F);
      
        
      oled.oled();  
     // delay(100);
     // drawStr(40, 30, time);
      //drawStr(20, 50, "4Mirrortech");
     // oled.oled();
    // oled.startscrollleft(0x00, 0x0F);
      delay(200);*/
      ultra2 = 0;
      sum3=0;
      Serial.println();
      delay(1000);
      }

     }
}
void else_case()
      {
      oled.clearDisplay();  /* Clear oled */
       delay(10);
        digitalWrite(ledRed, HIGH);
      delay(2);
      digitalWrite(ledGreen, LOW);
     // digitalWrite(led, HIGH);
     // delay(2);
     // digitalWrite(led, LOW);
       char time[30];
      //Serial.println("else case");
      oled.setCursor(6, 0);
     // sprintf(time, "%02d:%02d:%02d", h, m, s);
      //drawStr(20, 30, "Kushal");
      oled.setTextSize(5);
     // oled.setCursor(0,0);
      oled.print("AURA");
       oled.setCursor(29, 45);
     // sprintf(time, "%02d:%02d:%02d", h, m, s);
      //drawStr(20, 30, "Kushal");
      oled.setTextSize(2.5);
     // oled.setCursor(0,0);
      oled.print("MITOXY");
      oled.cp437(true);
      oled.display();  
      delay(700);
      oled.clearDisplay();
      delay(10);
      oled.setCursor(0, 10);
     // sprintf(time, "%02d:%02d:%02d", h, m, s);
      //drawStr(20, 30, "Kushal");
      oled.setTextSize(2);
     // oled.setCursor(0,0);
      oled.print("Powered by");
      //oled.setCursor(0, 30);
     // sprintf(time, "%02d:%02d:%02d", h, m, s);
      //drawStr(20, 30, "Kushal");
      oled.setCursor(0, 42);
      oled.setTextSize(3);
     // oled.setCursor(0,0);
      oled.print("4Mirror");
      oled.cp437(true);
      //oled.startscrollleft(0x00, 0x0F);
      oled.display();
      delay(700);
     // delay(100);  
     
     

      
      ultra2 = 0;
      sum3=0;
            HR = 0.0;
                      SPO2 = 0.0;
                      HR_SUM = 0.0;
                      SPO2_SUM = 0.0;
                      FINAL_HR_SUM=0;
                              FINAL_SPO2_SUM=0;
      
     // delay(100);
     // drawStr(40, 30, time);
      //drawStr(20, 50, "4Mirrortech");
     // oled.oled();
    // oled.startscrollleft(0x00, 0x0F);
     // delay(200);
      //Serial.println();
      //delay(500);
  }
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }

}
/*
         char time[30];
      oled.clearoled();
      oled.setCursor(6, 0);
     // sprintf(time, "%02d:%02d:%02d", h, m, s);
      //drawStr(20, 30, "Kushal");
      oled.setTextSize(5);
     // oled.setCursor(0,0);
      oled.print("AURA");
      oled.setCursor(25, 40);
     // sprintf(time, "%02d:%02d:%02d", h, m, s);
      //drawStr(20, 30, "Kushal");
      oled.setTextSize(2.95);
     // oled.setCursor(0,0);
      oled.print("4");
      oled.setCursor(38, 48);
      oled.setTextSize(1.95);
     // oled.setCursor(0,0);
      oled.print("MirrorTech");
      oled.cp437(true);
    //  oled.startscrollleft(0x00, 0x0F);
      oled.oled(); 
      */
 
