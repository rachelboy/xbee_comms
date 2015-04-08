#include <SoftwareSerial.h>

const int write_estop = A0; // the pin that the LED is attached to
const int read_estop = A1; // a pin attached to the input to the relay (should pull low)
int incomingByte;      // a variable to read incoming serial data into
unsigned long lastPing = 0;

byte okAck[] = {0x3B, 0x29, 0x0A};
byte stopAck[] = {0x3A, 0x4F, 0x0A};

SoftwareSerial remoteSerial(10,11); // RX, TX

void deadMansSwitch(){
  unsigned long time = millis();
  if(((time>>5)&1) == 1) {
    digitalWrite(write_estop, HIGH);
  } else {
    digitalWrite(write_estop, LOW);
  }
}

void eStop() {
  digitalWrite(write_estop, LOW);
}

void alertEStop() {
  Serial.print('STOP');
}

void setup() {
  Serial.begin(9600);
  remoteSerial.begin(19200);
  pinMode(write_estop, OUTPUT);
  pinMode(read_estop, INPUT);
}

void loop() {
   if((lastPing != 0) && (millis()-lastPing < 1000)) {
     deadMansSwitch();
   }
  
  if (remoteSerial.available() > 0) {
    incomingByte = remoteSerial.read();
    if (incomingByte == 'A') {
      lastPing = millis();
      remoteSerial.write(okAck,3);
    } 
    if (incomingByte == 'B') {
      eStop();
      lastPing = 0;
      remoteSerial.write(stopAck,3);
    }
    
  }
  
  if (Serial.available() > 0) {
    Serial.println(Serial.read()):
  }
  
//  hook a pin up to the signal going to the relay 
//  so that we can report if the eStop is engaged
//  (should pull low when floating)
//  if (analogRead(read_estop) < 100) {
//    alertEStop();
//  }
}
