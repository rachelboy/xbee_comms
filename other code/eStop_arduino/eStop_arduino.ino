const int write_estop = A0; // the pin that the LED is attached to
const int read_estop = A1; // a pin attached to the input to the relay (should pull low)
int incomingByte;      // a variable to read incoming serial data into
unsigned long lastPing = 0;

byte okAck[3];
byte stopAck[3];

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
  Serial.begin(19200);
  pinMode(write_estop, OUTPUT);
  pinMode(read_estop, INPUT);
  okAck[0] = 0x3B;
  okAck[1] = 0x29;
  okAck[2] = 0x0A;
  stopAck[0] = 0x3A;
  stopAck[1] = 0x4F;
  stopAck[2] = 0x0A;
}

void loop() {
   if((lastPing != 0) && (millis()-lastPing < 1000)) {
     deadMansSwitch();
   }
  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'A') {
      lastPing = millis();
      Serial.write(okAck,3);
    } 
    if (incomingByte == 'B') {
      eStop();
      lastPing = 0;
      Serial.write(stopAck,3);
    }
  }
  
//  hook a pin up to the signal going to the relay 
//  so that we can report if the eStop is engaged
//  (should pull low when floating)
//  if (analogRead(read_estop) < 100) {
//    alertEStop();
//  }
}
