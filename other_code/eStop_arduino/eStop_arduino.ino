#include <SoftwareSerial.h>

const int write_estop = 13; // the pin that the LED is attached to
const int read_estop = A1; // a pin attached to the input to the relay (should pull low)
int incomingByte;      // a variable to read incoming serial data into
unsigned long lastPing = 0;
unsigned long lastAlert = 0;
unsigned long time;

byte okAck[] = {0x3B, 0x29, 0x0A}; // ;)
byte stopAck[] = {0x3A, 0x4F, 0x0A}; // :O
byte estopAlert[] = {0x53, 0x54, 0x4F, 0x50, 0x0A}; // STOP

SoftwareSerial remoteSerial(10,11); // RX, TX

void deadMansSwitch(){
  time = millis();
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
  time = millis();
  if (time - lastAlert > 2000 || lastAlert == 0){
    lastAlert = time;
    Serial.write(estopAlert, 5);
    remoteSerial.write(estopAlert, 5);
  }
}

void setup() {
  Serial.begin(9600);
  remoteSerial.begin(19200);
  pinMode(write_estop, OUTPUT);
  pinMode(read_estop, INPUT);
  
  Serial.println("howdy");
  remoteSerial.println("hi");
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
  
//  hook a pin up to the signal going to the relay 
//  so that we can report if the eStop is engaged
//  (should pull low when floating)
  if (analogRead(read_estop) < 100) {
    alertEStop();
  }
}
