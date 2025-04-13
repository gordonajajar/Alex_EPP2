#include <avr/io.h>

#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>


#include "constants.h"

//COLOR SENSOR
#define  s0 40       //Module pins wiring
#define s1 38
#define s2 41
#define s3 39
#define  out 43

#define TRIG_PIN PC2  // Define trigger pin
#define ECHO_PIN PC4  // Define echo pins
                      
int Red=0; int Green=0; int Blue=0;


// Map 0–180° to OCR2B values (for ~1ms to ~2ms pulse)
uint8_t angleToOCR(uint8_t angle) {
  return (angle * (250 - 125)) / 180 + 125;
}

void closeClaw() {
  for (int angle = 180; angle >= 0; angle -= 10) {
    OCR2B = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
  for (int angle = 0; angle <= 180; angle += 10) {
    OCR2A = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
}
void openClaw() {
  for (int angle = 180; angle >= 0; angle -= 10) {
    OCR2A = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
  for (int angle = 0; angle <= 180; angle += 10) {
    OCR2B = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
}

float GetDist(){ //return float of distance
  PORTC &= ~(1 << TRIG_PIN);
  _delay_us(2);
  PORTC |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTC &= ~(1 << TRIG_PIN);

  // Measure pulse width
  while (!(PINC & (1 << ECHO_PIN))); // Wait for HIGH
  int count = 0;
  while (PINC & (1 << ECHO_PIN)) {
    _delay_us(1);
    count++;
    if (count > 2000){
      return -1;
    }
  }
  return count/58.0;

}

unsigned long* GetColor() {

    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    delay(20);
    digitalWrite(s3, HIGH);
    Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    delay(20);
    digitalWrite(s2, HIGH);
    Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    delay(20);

    Serial.print("Red: ");
    Serial.print(Red);
    Serial.print(", ");

    Serial.print("Green: ");
    Serial.print(Green);
    Serial.print(", ");

    Serial.print("Blue: ");
    Serial.print(Blue);
    Serial.println();
}


void GetUltra(){ 
  // Trigger pulse

  int toofar = 0;
  int ans[5];
  for (int i = 0; i < 10; i += 1){
    if (toofar > 3){
      Serial.println("Too far");
      return;
    }
    ans[i] = GetDist();
    if (ans[i] == -1){
      toofar += 1;
    }
  }

  Serial.print("Distances: ");
  for (int i = 0; i < 10; i++) {
    Serial.print(ans[i]);
    if (i < 9) {
      Serial.print(", ");
    }
  }
  Serial.println();

}

void GetInfo(){
  GetUltra();
  GetColor();
}

void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b0011;
  PORTD |= 0b1100;

  DDRA |= (1 << PA7);
  PORTA |= (1 << PA7);
  
}

void setupUltra(){
  DDRC |= (1 << TRIG_PIN);  // Set TRIG_PIN as output
  DDRC &= ~(1 << ECHO_PIN); // Set ECHO_PIN as input
}

void setupServo(){
  DDRH |= (1 << PH6);
  DDRB |= (1 << PB4);

  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);  // Non-inverting PWM on OC2B
  TCCR2B = (1 << CS22);                   // Prescaler = 64
}

void setupColor(){
  pinMode(s0,OUTPUT);    //pin modes
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(out,INPUT);

  digitalWrite(s0,HIGH);  //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100%  (recommended)
  digitalWrite(s1,HIGH); //LOW/LOW is off HIGH/LOW is 20% and  LOW/HIGH is  2%
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  EIMSK |= 0b1100;
  EICRA |= 0b10100000;
  EICRA &= 0b10101111;
}

void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}



void setup() {
  // put your setup code here, to run once:

  stop();
  cli();
  setupColor();
  setupSerial();
  enablePullups();
  setupServo();
  setupUltra();
  sei();

  Serial.println("ready for commands");
}


void loop() {
  if (Serial.available() > 0) {
    // Read the incoming command
    char command = Serial.read();
    
    // Process the command
    switch (command) {
      case 'W': // Forward
      case 'w':
        forward();
        break;
        
      case 'S': // Backward
      case 's':
        backward();
        break;
        
      case 'A': // Left
      case 'a': 
        ccw();
        break;
        
      case 'D': // Right
      case 'd':
        cw();
        break;
        
      case 'Q': // Stop
      case 'q':
        stop();
        break;
        
      case 'O': // Open claw
      case 'o':
        openClaw();
        break;
        
      case 'I': // Close claw
      case 'i':
        closeClaw();
        break;

      case 'p': // Get color and distance
      case 'P':
        GetInfo();
        break;
        
      case '\n': // Ignore newline characters
      case '\r':
        break;
        
      default:
        Serial.print("Unknown command: ");
        Serial.println(command);
    }
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

