#include <avr/io.h>

#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include "packet.h"
#include "constants.h"

#define ALEX_LENGTH 25.5
#define ALEX_BREADTH 14.9
#define ALEX_RADIUS 3
#define PI 3.141592654

//COLOR SENSOR
#define  s0 40       //Module pins wiring
#define s1 38
#define s2 41
#define s3 39
#define  out 43

#define TRIG_PIN PC2  // Define trigger pin
#define ECHO_PIN PC4  // Define echo pins
                      
int Red=0; int Green=0; int Blue=0;

float alexDiagonal = 0.0;
float alexCirc = 0.0;

volatile TDirection dir;
/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      2

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          2*PI*ALEX_RADIUS 

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Keep track whether we've moved a commanded distance
volatile unsigned long deltaDist;
volatile unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;


// Map 0–180° to OCR2B values (for ~1ms to ~2ms pulse)
uint8_t angleToOCR(uint8_t angle) {
  return (angle * (250 - 125)) / 180 + 125;
}

void openClaw() {
  for (int angle = 180; angle >= 0; angle -= 10) {
    OCR2B = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
  for (int angle = 0; angle <= 180; angle += 10) {
    OCR2A = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
}
void closeClaw() {
  for (int angle = 180; angle >= 0; angle -= 10) {
    OCR2A = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
  for (int angle = 0; angle <= 180; angle += 10) {
    OCR2B = angleToOCR(angle);
    _delay_ms(10); // small delay between steps
  }
}

float GetDist(){
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

    dbprintf("Red: %i, Green, %i, Blue, %i", Red, Green, Blue);
    // long arr[3];
    // arr[0] = Red;
    // arr[1] = Green;
    // arr[2] = Blue;
    // return arr;
}


void GetUltra(){ 
  // Trigger pulse

  int toofar = 0;
  int ans[5];
  for (int i = 0; i < 10; i += 1){
    if (toofar > 3){
      dbprintf("Too far");
      return;
    }
    ans[i] = GetDist();
    if (ans[i] == -1){
      toofar += 1;
    }
  }
  dbprintf("Distances: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i", ans[0], ans[1], ans[2], ans[3], ans[4], ans[5], ans[6], ans[7], ans[8], ans[9]);

}

// void getAvgUltra(){
//   int n = 0;
//   float sum = 0;
//   int toofar = 0;
//   while (n != 10){
//     if (toofar == 3){
//       Serial.println("too far");
//       return;
//     }
//     float val = GetUltra();
//     if (val == -1){
//       toofar += 1;
//     } else{
//       sum += val;
//       n += 1;
//     }
//   }
//   Serial.println(sum/10.0);
// }

unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (150.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed){
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = leftReverseTicksTurns + deltaTicks;

  ccw(ang, speed);
}

void right(float ang, float speed){
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;
  cw(ang, speed);
}

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

// void sendColorDist()
// {
//   // Implement code to send back a packet containing key
//   // information like leftTicks, rightTicks, leftRevs, rightRevs
//   // forwardDist and reverseDist
//   // Use the params array to store this information, and set the
//   // packetType and command files accordingly, then use sendResponse
//   // to send out the packet. See sendMessage on how to use sendResponse.
//   TPacket statusPacket;
//   statusPacket.packetType = PACKET_TYPE_RESPONSE;
//   statusPacket.command = RESP_STATUS;
//   statusPacket.data = 
//   statusPacket.params[0] = leftForwardTicks;
//   statusPacket.params[1] = rightForwardTicks;
//   statusPacket.params[2] = leftReverseTicks;
//   statusPacket.params[3] = rightReverseTicks;
//   statusPacket.params[4] = leftForwardTicksTurns;
//   statusPacket.params[5] = rightForwardTicksTurns;
//   statusPacket.params[6] = leftReverseTicksTurns;
//   statusPacket.params[7] = rightReverseTicksTurns;
//   statusPacket.params[8] = forwardDist;
//   statusPacket.params[9] = reverseDist;
//
//   sendResponse(&statusPacket);
// }
void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  long* arr = GetColor();
  statusPacket.params[0] = arr[0];
  statusPacket.params[1] = arr[1];
  statusPacket.params[2] = arr[2];

  //GetDist() returns -1 if too far. FLOAT
  int toofar = 0;
  for (int i = 3; i < 13; i += 2){ //3,5,7,9,11
    float dist = GetDist();
    unsigned long left = (unsigned long) dist;
    unsigned long right = (unsigned long) (dist*100)%100;
    statusPacket.params[i] = left;
    statusPacket.params[i+1] = right;
  }
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf (char* format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  dbprintf("COMMAND BAD");
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 19
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

// Functions to be called by INT2 and INT3 ISRs.
ISR(INT3_vect) //LEFT ENCODER
{
  if (dir == FORWARD){
    leftForwardTicks++;
  }
  else if (dir == BACKWARD){
    leftReverseTicks++;
  }
  else if (dir == LEFT){
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT){
    leftForwardTicksTurns++;
  }

  if (dir == FORWARD){
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == BACKWARD){
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  // dbprintf("Left Wheel Distance: ");
  // dbprintf(leftTicks*WHEEL_CIRC/COUNTS_PER_REV);
}

ISR(INT2_vect) //RIGHT ENCODER
{
  if (dir == FORWARD){
    rightForwardTicks++;
  }
  else if (dir == BACKWARD){
    rightReverseTicks++;
  }
  else if (dir == LEFT){
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT){
    rightReverseTicksTurns++;
  }
  // dbprintf("Right Wheel Distance: ");
  // dbprintf(rightTicks*WHEEL_CIRC/COUNTS_PER_REV);
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
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EIMSK |= 0b1100;
  EICRA |= 0b10100000;
  EICRA &= 0b10101111;

}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.


// Implement INT2 and INT3 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}

// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    case COMMAND_GET_COLOR:
        sendOK();
        GetColor();
        break;
    case COMMAND_GET_DIST:
        sendOK();
        GetUltra();
        break;
    case COMMAND_OPEN:
        sendOK();
        openClaw();
        break;
    case COMMAND_CLOSE:
        sendOK();
        closeClaw();
        break;
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
        break;

    case COMMAND_REVERSE:
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
        break;

    case COMMAND_TURN_LEFT:
        sendOK();
        left((double) command->params[0], (float) command->params[1]);
        break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right((double) command->params[0], (float) command->params[1]);
        break;

    case COMMAND_STOP:
        sendOK();
        stop();
        break;

    case COMMAND_GET_STATS:
        sendStatus();
        break;

    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
        break;

    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  cli();
  setupColor();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  setupServo();
  setupUltra();
  sei();
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      sendOK();
      break;
  }
}

void loop() {
// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

 //forward(0, 100);

// Uncomment the code below for Week 9 Studio 25

 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi
  
  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else if(result == PACKET_BAD) {
    sendBadPacket();
  }
  else if(result == PACKET_CHECKSUM_BAD){
    sendBadChecksum();
  } 
  
  if(deltaDist > 0)
  {
    if(dir==FORWARD){
      if(forwardDist > newDist){
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
    else if(dir == BACKWARD){
      if(reverseDist > newDist)
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
    else if(dir == STOP){
      deltaDist=0;
      newDist=0;
      stop();
    }
  }
  
  if (deltaTicks > 0){
    if(dir==LEFT){
      if(leftReverseTicksTurns >= targetTicks){
        deltaTicks=0;
        targetTicks=0;
        stop();
      }
    }
    else if(dir == RIGHT){
      if(rightReverseTicksTurns >= targetTicks){
        deltaTicks=0;
        targetTicks=0;
        stop();
      }
    }
    else if (dir == STOP){
      deltaTicks=0;
      targetTicks=0;
      stop();
    }
  }

}
