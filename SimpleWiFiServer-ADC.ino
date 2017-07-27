/*
  WiFi Web Server LED Blink

 A simple web server that lets you blink an LED via the web.
 This sketch will print the IP address of your WiFi Shield (once connected)
 to the Serial monitor. From there, you can open that address in a web browser
 to turn on and off the LED on pin 5.

 If the IP address of your shield is yourAddress:
 http://yourAddress/H turns the LED on
 http://yourAddress/L turns it off

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the Wifi.begin() call accordingly.

 Circuit:
 * WiFi shield attached
 * LED attached to pin 5

 created for arduino 25 Nov 2012
 by Tom Igoe

ported for sparkfun esp32 
31.01.2017 by Jan Hendrik Berlin
 
 */
#include "esp_system.h"
#include <WiFi.h>

const char* ssid     = "SSID";
const char* password = "PASSKEY";
int *aDcData0;
int *aDcData1;
int *aDcData2;
int *aDcData3;

long int total0,total1,total2,total3;
int SampleCount = 100;
int y,y2,record;

unsigned long ulMeasCount=0;
unsigned long ulNextMeas_ms=0;
//int  ReFresh = 10;
int siZe = 1000;

WiFiServer server(80);


hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}



void GetReading(){
   ulMeasCount++;
   if(ulMeasCount>(siZe-1)){ulMeasCount=1;clearArray();}
   for(int i=0;i<SampleCount;i++){
   total0 += analogRead(32);  // Read ADC
   total1 += analogRead(33); 
   total2 += analogRead(34); 
   total3 += analogRead(35); 
   }
   aDcData0[ulMeasCount] = total0 / SampleCount;  // Read ADC7
   aDcData1[ulMeasCount] = total1 / SampleCount;
   aDcData2[ulMeasCount] = total2 / SampleCount;
   aDcData3[ulMeasCount] = total3 / SampleCount;
   total0=0;
   total1=0;
   total2=0;
   total3=0;  
   }

void clearArray(){
for(int i=0;i<siZe;i++){
aDcData0[i]=0;
aDcData1[i]=0;
aDcData2[i]=0;
aDcData3[i]=0; 
}
}


void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);      // set the LED pin mode

    delay(10);

    // We start by connecting to a WiFi network

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
    
    server.begin();
aDcData0 = new int[siZe];
aDcData1 = new int[siZe];
aDcData2 = new int[siZe];
aDcData3 = new int[siZe];
clearArray();
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 2000, true);  // 20 milliseconds

  // Start an alarm
  timerAlarmEnable(timer);
}

int value = 0;
int val = 0;

void loop(){

  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    GetReading();
  }
  
/* if (millis()>=ulNextMeas_ms)  
  { 
     ulNextMeas_ms = millis()+ReFresh;  
     GetReading();  
  }*/ 


 WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    //val = hallRead();                       // Read Hall Sensor
    IPAddress ip = WiFi.localIP();          // IP to display
    FlashMode_t ideMode = ESP.getFlashChipMode();
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // the content of the HTTP response follows the header:
for(int loOp=32;loOp<36;loOp++){
  String out = "";
  char temp[100];
  int dIspV;
  if(loOp==32) { y = aDcData0[1] / 30; dIspV=aDcData0[ulMeasCount];}
  else if(loOp==33) { y = aDcData1[1] / 30; dIspV=aDcData1[ulMeasCount];}
  else if(loOp==34) { y = aDcData2[1] / 30; dIspV=aDcData2[ulMeasCount];}
  else if(loOp==35) { y = aDcData3[1] / 30; dIspV=aDcData3[ulMeasCount];}
           client.print("Last ADC Value Sampled on pin ");
           client.print(loOp);
           client.print("  : ");
           client.print(dIspV);
           client.print("<BR>");
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"1000\" height=\"150\">\n";
  out += "<rect width=\"1000\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";
  record = 2;
  for (int x = 1; x < ulMeasCount; x+=1) {
    if(loOp==32) { y2 = aDcData0[record] / 30;}
    else if(loOp==33) { y2 = aDcData1[record] / 30;}
    else if(loOp==34) { y2 = aDcData2[record] / 30;}
    else if(loOp==35) { y2 = aDcData3[record] / 30;}
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 1, 140 - y2);
    out += temp;
    y = y2;
    record++;
    client.print(out);
    out="";
  }
  
  out += "</g>\n</svg><BR>";

  client.print(out);
}
            client.print("<BR>Click <a href=\"/H\">here</a> turn the Onboard LED on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the Onboard LED off<br>");
           // client.print("Hall Sensor Value : ");
            //client.print(String(val));
            client.print("<BR>Local IP Address : ");
            client.print(ip);
            client.print("<BR>  Free RAM = ");
            client.print((uint32_t)system_get_free_heap_size() / 1024);
            client.print(" KB<BR>  SDK Version = ");
            client.print(ESP.getSdkVersion());
            client.print("<BR>");
            client.printf("  Flash Chip Mode = %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
            client.print("<BR>  Flash Size (IDE) = "); 
            client.print(ESP.getFlashChipSize()/1024);
            client.print(" KBytes<BR>  Flash Speed = ");
            client.print(ESP.getFlashChipSpeed()/1000000);
            client.print("MHz<BR>");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(LED_BUILTIN, HIGH);               // GET /H turns the LED on
            // put your main code here, to run repeatedly:
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}
