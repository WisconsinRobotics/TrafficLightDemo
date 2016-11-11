#define slow 700
#define fast 350
#define party 100

const int red =  8; const int yellow =  9; const int green =  10; //LED PINS
char in;

void setup() {
  Serial.begin(9600); //SERIAL
  pinMode(red, OUTPUT); pinMode(green, OUTPUT); pinMode(yellow, OUTPUT); //OUTPUT
  digitalWrite(red, LOW); digitalWrite(yellow, LOW); digitalWrite(green, LOW); //START ALL THREE ON
  delay(1000); //STARTUP ROUTINE
  digitalWrite(red, HIGH); digitalWrite(yellow, HIGH); digitalWrite(green, HIGH); //THEN OFF
}

void loop()
{
  if (Serial.available()){ //READ SERIAL MESSAGES
    in = Serial.read();
    while (Serial.available()) {Serial.read();} //CONSUME THE REST OF BUFFER
  }
  if (in == 'e') {slow_green();}
  if (in == 'f') {fast_green();}
  if (in == 'c') {slow_yellow();}
  if (in == 'd') {fast_yellow();}
  if (in == 'a') {slow_red();}
  if (in == 'b') {fast_red();}
  if (in == 'g') {party_mode();}
}

void slow_green(){
  digitalWrite(red, HIGH);
  digitalWrite(yellow, HIGH);
  digitalWrite(green, LOW);
  delay(slow);
  digitalWrite(green, HIGH);
  delay(slow);
}

void fast_green(){
  digitalWrite(red, HIGH);
  digitalWrite(yellow, HIGH);
  digitalWrite(green, LOW);
  delay(fast);
  digitalWrite(green, HIGH);
  delay(fast);
}

void slow_yellow(){
  digitalWrite(red, HIGH);
  digitalWrite(yellow, LOW);
  digitalWrite(green, HIGH);
  delay(slow);
  digitalWrite(yellow, HIGH);
  delay(slow);  
}

void fast_yellow(){
  digitalWrite(red, HIGH);
  digitalWrite(yellow, LOW);
  digitalWrite(green, HIGH);
  delay(fast);
  digitalWrite(yellow, HIGH);
  delay(fast);    
}

void slow_red(){
  digitalWrite(red, LOW);
  digitalWrite(yellow, HIGH);
  digitalWrite(green, HIGH);
  delay(slow);
  digitalWrite(red, HIGH);
  delay(slow);    
}

void fast_red(){
  digitalWrite(red, LOW);
  digitalWrite(yellow, HIGH);
  digitalWrite(green, HIGH);
  delay(fast);
  digitalWrite(red, HIGH);
  delay(fast);   
}

void party_mode(){
  digitalWrite(red, LOW);
  digitalWrite(yellow, LOW);
  digitalWrite(green, HIGH);
  delay(party);
  digitalWrite(red, LOW);
  digitalWrite(yellow, HIGH);
  digitalWrite(green, LOW);
  delay(party); 
  digitalWrite(red, HIGH);
  digitalWrite(yellow, LOW);
  digitalWrite(green, HIGH);
  delay(party); 
  digitalWrite(red, HIGH);
  digitalWrite(yellow, LOW);
  digitalWrite(green, LOW);
  delay(party); 
  digitalWrite(red, LOW);
  digitalWrite(yellow, LOW);
  digitalWrite(green, LOW);
  delay(party);
}

