int input_pins [7] = { 6, 3, 12, 7, 8,  9, 5};
int sensorOutput [7] = { 0, 0, 0, 0, 0, 0, 0 };
void setup() {
  //start serial connection
  Serial.begin(9600);

  //Enable input for sesor pins
  for (  int i = 0; i < 8; i = i +1 ) {
    pinMode(i, INPUT);
  }
}

void loop() {
  
  Serial.write(27); // ESC command
  Serial.print("[2J"); // clear screen command
  Serial.write(27);
  Serial.print("[H"); // cursor to home command

  for(  int i = 0; i < sizeof(input_pins)/sizeof(int); i = i + 1){
    sensorOutput[i] = digitalRead(input_pins[i]);
  }

  //Serial.write(12); //New page
  Serial.println("# Sensor values");
  for(  int i = 0; i < sizeof(sensorOutput)/sizeof(int); i = i + 1){
    Serial.print(" ");
    Serial.print( sensorOutput[i] );
  }
  Serial.println("\n");
  Serial.println("# Car status");
  Serial.println(" Speed: 0 m/s");
  Serial.println(" Turn-angle: 0 deg");
  delay(500);

}
