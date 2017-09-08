#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Adafruit_TCS34725.h>

#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define VBATPIN A9
#define RED 10
#define GRE 11
#define BLU 12


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);


void error(const __FlashStringHelper *err) {
  Serial.println(err);
  while (1);
}

void gotoSleep() {
  Serial.println("Going to sleep in one second...");
  delay(1000); 
  pinMode( 13, OUTPUT );
  while ( true ) { 
    digitalWrite( 13, ! digitalRead( 13 ) ) ; 
    delay( 100 ); 
  }  
}


void setup(void) {  

  pinMode(RED, OUTPUT);      pinMode(GRE, OUTPUT);    pinMode(BLU, OUTPUT);    /* Make led s output*/
  digitalWrite(RED, HIGH);  digitalWrite(GRE, HIGH);  digitalWrite(BLU, HIGH); /* Turn off all light */
  tcs.setInterrupt(false);

  float battery = getBattery(); /* get battery level */
  Serial.print("VBat: " ); Serial.println(battery);
  
  Serial.begin(115200);
  Serial.println("");
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Initialising the Bluefruit LE module: "));
  Serial.println(F("OK!")) ;
  
 
  if (!ble.begin(VERBOSE_MODE) ) { error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?")); }
  if (!ble.factoryReset() ){ error(F("Couldn't factory reset")); }
  ble.echo(false);
  ble.info();
  
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();
  
  
  ble.verbose(false);  // non need to debug anymore 
  while (!ble.isConnected()) { delay(100); } /* if not connected wait until conection established */
  Serial.println(F("**************** Connected BLE **************"));
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
  tcs.setInterrupt(true); /* Tuen off light on the sensor board */
  }

  
float getBattery() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

uint16_t atime, bat; 
uint16_t r,  g,  b,  c;
uint16_t r_r, r_g, r_b, r_c;
float rp_r, rp_g, rp_b, rp_c;
float gp_r, gp_g, gp_b, gp_c;
float bp_r, bp_g, bp_b, bp_c;
float ap_r, ap_g, ap_b, ap_c;

uint16_t g_r, g_g, g_b, g_c;
uint16_t b_r, b_g, b_b, b_c;
uint16_t a_r, a_g, a_b, a_c;
float ir, denom;


void loop(void) {    
  while(ble.available() ) {
    int cmd = ble.read();
    Serial.println("Input"+ (char) cmd);
    atime = (256 - int(TCS34725_INTEGRATIONTIME_50MS) ) * 2.4 *2;
    tcs.setInterrupt(true);    // turn off white light
    if( ((char) cmd == '1') ||  ((char) cmd == '2') || ((char) cmd == '3') || ((char) cmd == '4') ) {
      bat = getBattery()*100;

      tcs.setInterrupt(false);
      digitalWrite(RED, LOW); delay(atime); tcs.getRawData(&r_r, &r_g, &r_b, &r_c); digitalWrite(RED, HIGH);     // RED  light
      tcs.setInterrupt(true);
      
      ir = (r_r *1.0 + r_g*1.0 + r_b*1.0 - r_c*1.0);
           
      if(r_r == 0) r_r =1; 
      if(r_g == 0) r_g =1; 
      if(r_b == 0) r_b =1; 
      if(r_c == 0) r_c =1; 
      rp_r = (1.0 * r_r) * (1.0 - (ir / (1.0 * r_r) ));
      rp_g = (1.0 * r_g) * (1.0 - (ir / (1.0 * r_g) ));
      rp_b = (1.0 * r_b) * (1.0 - (ir / (1.0 * r_b) ));
      rp_c = (1.0 * r_c) * (1.0 - (ir / (1.0 * r_c) ));   
      denom = rp_r + rp_g + rp_b;
      r_r = (int) (1000.0 * rp_r / denom);
      r_g = (int) (1000.0 * rp_g / denom);
      r_b = (int) (1000.0 * rp_b / denom);
      r_c = (int) (rp_c / 10.0);
      Serial.print("r_r row");Serial.print( r_r, DEC); Serial.print("r_g row");Serial.print( r_g, DEC);  Serial.print("r_b row");Serial.print( r_b, DEC);    Serial.print("r_c row");Serial.println( r_c, DEC);

      tcs.setInterrupt(false);
      digitalWrite(GRE, LOW); delay(atime); tcs.getRawData(&g_r, &g_g, &g_b, &g_c); digitalWrite(GRE, HIGH);     // GRE  light
      tcs.setInterrupt(true);      

      ir = (g_r *1.0 + g_g * 1.0 + g_b *1.0 - g_c*1.0);
      
      if(g_r == 0) g_r =1; 
      if(g_g == 0) g_g =1; 
      if(g_b == 0) g_b =1; 
      if(g_c == 0) g_c =1; 
      gp_r = (1.0 * g_r) * (1.0 - (ir / (1.0 * g_r) ));
      gp_g = (1.0 * g_g) * (1.0 - (ir / (1.0 * g_g) ));
      gp_b = (1.0 * g_b) * (1.0 - (ir / (1.0 * g_b) ));
      gp_c = (1.0 * g_c) * (1.0 - (ir / (1.0 * g_c) ));     
      denom = gp_r + gp_g + gp_b;
      g_r = (int) (1000.0 * gp_r / denom);
      g_g = (int) (1000.0 * gp_g / denom);
      g_b = (int) (1000.0 * gp_b / denom);
      g_c = (int) (gp_c / 10.0);
      Serial.print("r_r row");Serial.print( r_r, DEC); Serial.print("r_g row");Serial.print( r_g, DEC);  Serial.print("r_b row");Serial.print( r_b, DEC);    Serial.print("r_c row");Serial.println( r_c, DEC);
       
      tcs.setInterrupt(false);
      digitalWrite(BLU, LOW); delay(atime); tcs.getRawData(&b_r, &b_g, &b_b, &b_c); digitalWrite(BLU, HIGH);     // BLU  light
      tcs.setInterrupt(true);
      
      ir = (b_r *1.0 + b_g * 1.0 + b_b * 1.0 - b_c * 1.0);
     
      if(b_r == 0) b_r =1; 
      if(b_g == 0) b_g =1; 
      if(b_b == 0) b_b =1; 
      if(b_c == 0) b_c =1; 
      bp_r = (1.0 * b_r) * (1.0 - (ir / (1.0 * b_r) ));
      bp_g = (1.0 * b_g) * (1.0 - (ir / (1.0 * b_g) ));
      bp_b = (1.0 * b_b) * (1.0 - (ir / (1.0 * b_b) ));
      bp_c = (1.0 * b_c) * (1.0 - (ir / (1.0 * b_c) )); 
      denom = bp_r + bp_g + bp_b;
      b_r = (int) (1000.0 * bp_r / denom);
      b_g = (int) (1000.0 * bp_g / denom);
      b_b = (int) (1000.0 * bp_b / denom);
      b_c = (int) (bp_c / 10.0);

      
      digitalWrite(RED, LOW); 
      digitalWrite(GRE, LOW); 
      digitalWrite(BLU, LOW); 
      tcs.setInterrupt(false);
      delay(atime); tcs.getRawData(&a_r, &a_g, &a_b, &a_c);   
      tcs.setInterrupt(true);
      digitalWrite(RED, HIGH); 
      digitalWrite(GRE, HIGH); 
      digitalWrite(BLU, HIGH); 
      
      ir = (a_r *1.0 + a_g * 1.0 + a_b * 1.0 - a_c * 1.0);
      if(a_r == 0) a_r =1; 
      if(a_g == 0) a_g =1; 
      if(a_b == 0) a_b =1; 
      if(a_c == 0) a_c =1; 
      ap_r = (1.0 * a_r) * (1.0 - (ir / (1.0 * a_r) ));
      ap_g = (1.0 * a_g) * (1.0 - (ir / (1.0 * a_g) ));
      ap_b = (1.0 * a_b) * (1.0 - (ir / (1.0 * a_b) ));
      ap_c = (1.0 * a_c) * (1.0 - (ir / (1.0 * a_c) )); 
      
      denom = ap_r + ap_g + ap_b;
      a_r = (int) (1000.0 * ap_r / denom);
      a_g = (int) (1000.0 * ap_g / denom);
      a_b = (int) (1000.0 * ap_b / denom);
      a_c = (int) (ap_c / 10.0);
      
      
      ble.print("r_r");ble.print( r_r, DEC); ble.print("r_g");ble.print( r_g, DEC);  ble.print("r_b");ble.print( r_b, DEC);    ble.print("r_c");ble.print( r_c, DEC);
      ble.print("g_r");ble.print( g_r, DEC); ble.print("g_g");ble.print( g_g, DEC);  ble.print("g_b");ble.print( g_b, DEC);    ble.print("g_c");ble.print( g_c, DEC);
      ble.print("b_r");ble.print( b_r, DEC); ble.print("b_g");ble.print( b_g, DEC);  ble.print("b_b");ble.print( b_b, DEC);    ble.print("b_c");ble.print( b_c, DEC);
      ble.print("a_r");ble.print( a_r, DEC); ble.print("a_g");ble.print( a_g, DEC);  ble.print("a_b");ble.print( a_b, DEC);    ble.print("a_c");ble.print( a_c, DEC);
      ble.print("cmd");ble.print( cmd, DEC); ble.print("pow");ble.print( bat, DEC);  ble.print("|");
      
      Serial.println( "" );
      Serial.println( "" );
      Serial.print("r_r");Serial.print( r_r, DEC); Serial.print("r_g");Serial.print( r_g, DEC);  Serial.print("r_b");Serial.print( r_b, DEC);    Serial.print("r_c");Serial.println( r_c, DEC);
      Serial.print("g_r");Serial.print( g_r, DEC); Serial.print("g_g");Serial.print( g_g, DEC);  Serial.print("g_b");Serial.print( g_b, DEC);    Serial.print("g_c");Serial.println( g_c, DEC);
      Serial.print("b_r");Serial.print( b_r, DEC); Serial.print("b_g");Serial.print( b_g, DEC);  Serial.print("b_b");Serial.print( b_b, DEC);    Serial.print("b_c");Serial.println( b_c, DEC);
      Serial.print("a_r");Serial.print( a_r, DEC); Serial.print("a_g");Serial.print( a_g, DEC);  Serial.print("a_b");Serial.print( a_b, DEC);    Serial.print("a_c");Serial.println( a_c, DEC);
      Serial.print("cmd");Serial.print( cmd, DEC); Serial.print("pow");Serial.print( bat, DEC);  Serial.print("|");
      Serial.println("Output"+ (char) cmd);
    }  
   
    if ((char) cmd == '8') {
      digitalWrite(13, HIGH);
      ble.reset();
      delay(1000);
      digitalWrite(13, LOW);
      Serial.print("8:");
    }

    if((char) cmd == '9') {
      bat = getBattery()*100;
      ble.print("cmd");ble.print( cmd, DEC); ble.print("pow");ble.print( bat, DEC);  ble.print("|");
      Serial.print("9");      
    }
  }  
}
