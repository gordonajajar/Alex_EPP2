#include <avr/io.h>

#include <serialize.h>
#include <stdarg.h>
#include <math.h>

#define TRIG_PIN PC2  // Define trigger pin
#define ECHO_PIN PC4  // Define echo pins

void GetUltra(){
  // Trigger pulse
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
    }

    // Convert pulse width to distance (speed of sound = 343 m/s)
    Serial.print("Ultrasonic Distance: ");  // Distance in cm
    Serial.println(count/58);
}

void setupUltra(){
  DDRC |= (1 << TRIG_PIN);  // Set TRIG_PIN as output
  DDRC &= ~(1 << ECHO_PIN); // Set ECHO_PIN as input
}

void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}


void setup() {
  // put your setup code here, to run once:
  DDRA |= (1 << PA7);

  cli();
  setupSerial();
  setupUltra();
  sei();
}

void loop() {
  GetUltra();
  delay(1000);
}
