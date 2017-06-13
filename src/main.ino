// #define ENCODERA_PIN 25
// #define ENCODERB_PIN 26
// #define ENCODER_BUT_PIN 15
// volatile int16_t encoder0Pos = 0;

// void read_encoder_ISR()
// {
//     static uint8_t old_AB = 0;
//     // grey code
//     // http://hades.mech.northwestern.edu/index.php/Rotary_Encoder
//     // also read up on 'Understanding Quadrature Encoded Signals'
//     // https://www.pjrc.com/teensy/td_libs_Encoder.html
//     // another interesting lib: https://github.com/0xPIT/encoder/blob/arduino/ClickEncoder.cpp
//     static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

//     old_AB <<= 2;
//     old_AB |= ((digitalRead(ENCODERB_PIN))?(1<<1):0) | ((digitalRead(ENCODERA_PIN))?(1<<0):0);
//     encoder0Pos += ( enc_states[( old_AB & 0x0f )]);
// }
// int16_t read_encoder()
// {
//     return encoder0Pos;
// }

// int8_t encoder_changed() {
//     static int16_t old_encoder0Pos = 0;
//     int8_t encoder0Diff = encoder0Pos - old_encoder0Pos;

//     old_encoder0Pos = encoder0Pos;
//     return encoder0Diff;
// }
// void setup(){
// Serial.begin(115200);
// Serial.println("Serial Begin");
//     pinMode (ENCODERA_PIN, INPUT_PULLUP);
//     pinMode (ENCODERB_PIN, INPUT_PULLUP);
//     attachInterrupt(ENCODERA_PIN, read_encoder_ISR, CHANGE);
//     attachInterrupt(ENCODERB_PIN, read_encoder_ISR, CHANGE);
// }

// void loop(){

//     Serial.println("Enable rotary encoder ISR:");
//     // Initialize rotary encoder reading and decoding

//     if (encoder_changed()) {
// 	Serial.print ("Encoder val: ");
// 	Serial.println (read_encoder());
//     }
//     delay(100);
//     }

#include "max6675.h"
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "NewFonts.h"
int oledSDA = 22;
int oledSCL = 21;
int thermoDO = 27;
int thermoCS = 26;
int thermoCLK = 25;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
SSD1306  display(0x3c, oledSDA, oledSCL);

int vccPin = 33;
int gndPin = 32;
  
void setup() {
  Serial.begin(9600);
  // use Arduino pins 
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
  display.init();
  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
  display.flipScreenVertically();
}

void loop() {
  // basic average readout test

  float temp1 = thermocouple.readFahrenheit();
  float temp2 = thermocouple.readFahrenheit();
  float temp3 = thermocouple.readFahrenheit();
  float tempavg =0;

  tempavg = (temp1+temp2+temp3)/3;
  int intTempAvg =(int)(tempavg < 0 ? (tempavg - 0.5) : (tempavg + 0.5));
  
   String stringTemp = "500 F";
   stringTemp = String(intTempAvg) + "° F";
   //Serial.print("°F = ");
  // Serial.println(intTempAvg);
   display.clear();
   display.setTextAlignment(TEXT_ALIGN_CENTER);
   display.setFont(Roboto_Medium_26);
   display.drawString(64, 22, stringTemp);
   Serial.println(stringTemp);
   display.display();
   delay(1000);
}