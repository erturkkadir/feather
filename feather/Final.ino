#include <Arduino.h>
#include <SPI.h>
//#include <SoftwareSerial.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Adafruit_TCS34725.h>
//#include <Adafruit_SleepyDog.h>


#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define VBATPIN A9



Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
  //SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
  //Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);



void error(const __FlashStringHelper *err) {
  Serial.println(err);
  while (1);
}

void gotoSleep() {
  Serial.println("Going to sleep in one second...");
  delay(1000);
  //int sleepMS = Watchdog.sleep();
  pinMode( 13, OUTPUT );
  while ( true ) { 
    digitalWrite( 13, ! digitalRead( 13 ) ) ; 
    delay( 100 ); 
  }  
}


void setup(void) {  
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);

  float battery = getBattery();
  Serial.print("VBat: " ); Serial.println(battery);

  /*
  Serial.begin(9600);
  Serial.println("");
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Initialising the Bluefruit LE module: "));
  Serial.println( F("OK!") );
  */
  
  if (!ble.begin(VERBOSE_MODE) ) { error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?")); }
  if (!ble.factoryReset() ){ error(F("Couldn't factory reset")); }
   ble.echo(false);
  //Serial.println("Requesting Bluefruit info:");
  ble.info();
  /*
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();
  */
  
  ble.verbose(false);  // debug info is a little annoying after this point!
  while (!ble.isConnected()) { delay(100); }
  //Serial.println(F("**************** Connected **************"));
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");   
  }      
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println(F("******************************"));
  tcs.setInterrupt(false);
  }

  
float getBattery() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

int n = 1;
uint16_t atime; 
uint16_t r,  g,  b,  c,  t,  l;
float ir;
float fR, fG, fB, fC;

void loop(void) {    
  while(ble.available() ) {
    int ch = ble.read();
    //Serial.println((char) ch);
    atime = int(TCS34725_INTEGRATIONTIME_24MS);
    if((char) ch == '1') { // Send mobile the color value
     
      tcs.setInterrupt(false);    // turn on white light
      digitalWrite(11, LOW);      // turn on left  light
      digitalWrite(12, LOW);      // turn on right light
      //tcs.getRawData(&r, &g, &b, &c);  // clear out
      for(int i=0; i<n; i++) {    // read n times        
        delay((256 - atime) * 2.4 *2);                // 24 * 2 -+ 2  
        tcs.getRawData(&r, &g, &b, &c);  
        t = tcs.calculateColorTemperature(r, g, b);  
        l = tcs.calculateLux(r, g, b);        
      }
    digitalWrite(11, HIGH);     // turn off left light
    digitalWrite(12, HIGH);     // turn off right light
    tcs.setInterrupt(true);     // turn off white light
    Serial.print("R:"); Serial.println(r, DEC);
    Serial.print("G:"); Serial.println(g, DEC);
    Serial.print("B:"); Serial.println(b, DEC);
    Serial.print("C:"); Serial.println(c, DEC);
    Serial.print("t:"); Serial.println(t, DEC);
    Serial.print("l:"); Serial.println(l, DEC);
    
    
    ble.print("r");ble.print( r, DEC); 
    ble.print("g");ble.print( g, DEC);
    ble.print("b");ble.print( b, DEC);
    ble.print("c");ble.print( c, DEC);
    ble.print("t");ble.print( t, DEC);
    ble.print("l");ble.print( l, DEC); 
    ble.print("|");         
    }
    
    if (ch=='2') {
      digitalWrite(13, HIGH);
      ble.reset();
      delay(1000);
      digitalWrite(13, LOW);
    }

    if(ch=='3') {
      float bat = getBattery();
      ble.print("Battery");ble.print( bat, DEC); 
      ble.print("|");            
    }
  }
}



