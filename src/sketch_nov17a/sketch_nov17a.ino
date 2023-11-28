void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int incomingByte = 0; // for incoming serial data
int analogPin = A3;

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }
  int val = analogRead(analogPin);  // read the input pin
  Serial.println(val);          // debug value
  delay(1000);
}
