// Clock and data pins are pins from the hardware SPI, you cannot choose them yourself.
// Data pin is MOSI (Uno and earlier: 11, Leonardo: ICSP 4, Mega: 51, Teensy 2.0: 2, Teensy 2.0++: 22) 
// Clock pin is SCK (Uno and earlier: 13, Leonardo: ICSP 3, Mega: 52, Teensy 2.0: 1, Teensy 2.0++: 21)

// You can choose the latch pin yourself.
const int ShiftPWM_latchPin=8;

// ** uncomment this part to NOT use the SPI port and change the pin numbers. This is 2.5x slower **
// #define SHIFTPWM_NOSPI
// const int ShiftPWM_dataPin = 11;
// const int ShiftPWM_clockPin = 13;


// If your LED's turn on if the pin is low, set this to true, otherwise set it to false.
const bool ShiftPWM_invertOutputs = false;

// You can enable the option below to shift the PWM phase of each shift register by 8 compared to the previous.
// This will slightly increase the interrupt load, but will prevent all PWM signals from becoming high at the same time.
// This will be a bit easier on your power supply, because the current peaks are distributed.
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

// Here you set the number of brightness levels, the update frequency and the number of shift registers.
// These values affect the load of ShiftPWM.
// Choose them wisely and use the PrintInterruptLoad() function to verify your load.
unsigned char maxBrightness = 255;
unsigned char pwmFrequency = 75;
unsigned int numRegisters = 3;
unsigned int numOutputs = numRegisters*8;
unsigned int numRGBLeds = numRegisters*8/3;

int ptr = -1;
byte nextChar;
byte cmdBuf[5];
int colors[][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
int freqs[] = {0,0,0,0,0,0,0,0};
boolean fades[] = {false, false, false, false, false, false, false, false};

void parseCmd() {
  int led = (int)(cmdBuf[0]);
  colors[led][0] = (int)(cmdBuf[1]);
  colors[led][1] = (int)(cmdBuf[2]);
  colors[led][2] = (int)(cmdBuf[3]);
  int f = (int)(cmdBuf[4]);
  if(f < 128) {
    freqs[led] = f;
    fades[led] = false;
  } else {
    freqs[led] = f-128;
    fades[led] = true;
  }
}

void updateLights(){
  for(unsigned int led=0; led<numRGBLeds; led++){
    if(freqs[led] == 0){
      ShiftPWM.SetRGB(led,colors[led][0],colors[led][1],colors[led][2]);
    } else {
    unsigned long time = millis();
    if(!(fades[led])){
//      time = time>>freqs[led]; //divide by the appropriate factor of 2
      if(((time>>(freqs[led]))&1) == 0) {
        ShiftPWM.SetRGB(led,colors[led][0],colors[led][1],colors[led][2]);
      } else {
        ShiftPWM.SetRGB(led,0,0,0);
      }
    } else {
      int max_time = pow(2, (freqs[led])-1) - 1;
      int current_time = time&max_time;
      float perc;
      if ((time>>(freqs[led]-1)&1) == 0) {
        //fading in
        perc = (float)current_time/(float)max_time;
      } else {
        //fading out
        perc = 1 - (float)current_time/(float)max_time;
      }
      ShiftPWM.SetRGB(led,(int)(perc*colors[led][0]),(int)(perc*colors[led][1]),(int)(perc*colors[led][2]));
      }
    }
  }
}



void setup() {
  while(!Serial){
    delay(100); 
  }
  Serial.begin(9600);
  
  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);

  // SetPinGrouping allows flexibility in LED setup. 
  // If your LED's are connected like this: RRRRGGGGBBBBRRRRGGGGBBBB, use SetPinGrouping(4).
  ShiftPWM.SetPinGrouping(2); //This is the default, but I added here to demonstrate how to use the funtion
  
  ShiftPWM.Start(pwmFrequency,maxBrightness);
}

void loop() {
  if (Serial.available() > 0) {
    nextChar = Serial.read();
    Serial.println(nextChar);
    if (nextChar == 254 && ptr == -1) {
      ptr = 0;
    } else if (nextChar == 255 && ptr==5) {
      ptr = -1;
      for (int i=0; i<5; i++) {
        Serial.println(cmdBuf[i]);
      }
      parseCmd();
        Serial.println(freqs[0]);
        Serial.println(fades[0]);
    } else if (ptr > -1 && ptr < 5) {
      cmdBuf[ptr] = nextChar;
      ptr++;
    } else {
      Serial.println("Something is not right");
      ptr = -1;
      while(Serial.available() > 0) {
        Serial.read();
      }
    }
  }
  

  updateLights();
  

}
