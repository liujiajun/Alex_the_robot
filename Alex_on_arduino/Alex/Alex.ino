#include <PID_v1.h>
#include <serialize.h>
#include <math.h>
#include <MsTimer2.h>
#include "packet.h"
#include "constants.h"
#define CS_S0 A4
#define CS_S1 A3
#define CS_S2 A1
#define CS_S3 A0
#define CS_Out A2
#define FRONT_IR A5
//RGB
int red;
int green;
int blue;

double sp=0, in, out;
PID pid(&in, &out, &sp,10,10,0,DIRECT);

double lspeed=0, rspeed=0;
unsigned long lcnt=0, rcnt=0;
double lPeriod, rPeriod, llast=0, rlast=0;
double lLastImcompletePulse=0, rLastImcompletePulse=0;
int val;

double in2, out2, sp2=0;
PID pid2(&in2, &out2, &sp2, 3, 0, 0, DIRECT);

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV     195

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          24

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  9  // Right reverse pin

//PI,
#define PI 3.141592654

//Alex's length and breadth in cm
#define ALEX_LENGTH  18
#define ALEX_BREADTH 11

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

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

// Varibles to keep track of whether we have moved a commanded distance
volatile unsigned long deltaDist;
volatile unsigned long newDist;

volatile unsigned long deltaTicks;
volatile unsigned long targetTicks;

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


// we create a new packet with parameters 
// to store the information on the 
// right side of '=' signs, and send back to 
// the Pi
void sendStatus() 
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
}
void sendColor()
{
  const int N = 10;
  TPacket colorPacket;
  colorPacket.packetType = PACKET_TYPE_RESPONSE;
  colorPacket.command = RESP_COLOR;
  int rsum=0, gsum=0, bsum=0;
  for(int i=0; i<N; i++){
    digitalWrite(CS_S2,LOW);
    digitalWrite(CS_S3,LOW);
    rsum += pulseIn(CS_Out, LOW);
    delay(40);
    digitalWrite(CS_S2,HIGH);
    digitalWrite(CS_S3,HIGH);
    gsum += pulseIn(CS_Out, HIGH);
    delay(40);
    digitalWrite(CS_S2,LOW);
    digitalWrite(CS_S3,HIGH);
    bsum += pulseIn(CS_Out, HIGH);
    delay(40);
}
  colorPacket.params[0] = rsum/N;
  colorPacket.params[1] = gsum/N;
  colorPacket.params[2] = bsum/N;
  sendResponse (&colorPacket);
}
void sendIR(){
  TPacket IRPacket;
  IRPacket.packetType = PACKET_TYPE_RESPONSE;
  IRPacket.command = RESP_IR;
  IRPacket.params[0] = digitalRead(FRONT_IR);
  IRPacket.params[1] = 0;
  IRPacket.params[2] = 0;
  sendResponse (&IRPacket);
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
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 

  DDRD  &= 0b11110011;
  PORTD |= 0b00001100;


}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  //PID
  lcnt++;
  lPeriod = micros()-llast;
  llast = micros();
  
  if (dir == FORWARD) {
    leftForwardTicks++;
  } else if (dir == BACKWARD) {
    leftReverseTicks++;
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }

  if (dir == FORWARD) 
    forwardDist = (unsigned long) ((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  else if (dir == BACKWARD)
    reverseDist = (unsigned long) ((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC); 

   //Serial.print("LEFT: "); Serial.println(leftTicks);
} 

void rightISR()
{
  //PID
  rcnt++;
  rPeriod = micros()-rlast;
  rlast = micros();
  
  if (dir == FORWARD) {
    rightForwardTicks++;
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == LEFT) {
    rightReverseTicksTurns++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  //pin2,3 falling edge
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

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
  Serial.begin(57600);
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

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  //Code to tell us how far to move
  if (dist >0)
      deltaDist = dist;
  else
      deltaDist = 9999999;
  newDist = forwardDist + deltaDist; 
  dir = FORWARD; 
  val = speed;//pwmVal(speed);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  //Code to tell us how far to move
  if (dist >0)
      deltaDist = dist;
  else
      deltaDist = 9999999;
  newDist = reverseDist + deltaDist;
  dir = BACKWARD;
  val = speed;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.

unsigned long computeDeltaTicks (float ang)
{
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV ) / (360.0 * WHEEL_CIRC)) * 0.39;
  return ticks;
}


void left(float ang, float speed)
{
  val = speed;
  dir = LEFT;
  if (ang == 0)
      deltaTicks = 99999999;
  else 
      deltaTicks = computeDeltaTicks(ang);
  
  targetTicks = leftReverseTicksTurns + deltaTicks;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  val = speed;

  if (ang == 0)
      deltaTicks = 99999999;
  else 
      deltaTicks = computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP; ///xxxxx
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
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
  leftReverseTicks=0; 
  rightReverseTicks=0;
  
  leftForwardTicksTurns=0; 
  rightForwardTicksTurns=0;
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
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance/angle, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break; 

    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_GET_COLOR:
        sendColor();
      break;
    case COMMAND_GET_IR:
        sendIR();
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;

    case COMMAND_GET_STATS:
        sendStatus(); //
      break;
    
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command -> params[0]); // clear the counters in param[0], the distance/angle only, not clearing the speed
      break;  
    
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
void goPID(){
  int lm,rm;
  if (dir == FORWARD || dir == BACKWARD){
      double lIncompletePulse = (micros()-llast) / lPeriod;
      int lcompletePulse = lcnt;
      lspeed = (1-lLastImcompletePulse + lcnt-1 + lIncompletePulse);
      lLastImcompletePulse = lIncompletePulse;
      double rIncompletePulse = (micros()-rlast) / rPeriod;
      int rcompletePulse = rcnt;
      rspeed = (1-rLastImcompletePulse + rcnt-1 + rIncompletePulse);
      rLastImcompletePulse = rIncompletePulse;
      if(lcnt>5 && rcnt>5) {
          in = lspeed - rspeed;
          pid.Compute();
      } else {
          if (dir == BACKWARD) { analogWrite(LF, 0); analogWrite(RF, 0); analogWrite(LR,160); analogWrite(RR, 160); }    
          if (dir == FORWARD) { analogWrite(LF, 160); analogWrite(RF, 160); analogWrite(LR,0); analogWrite(RR, 0); }    
      }
      //in = lcnt-rcnt;
      //pid.Compute();
      //Serial.print("cnt"); 
      //Serial.print(lcnt); Serial.print(" "); Serial.print(rcnt);  Serial.print("  speed: ");
      //Serial.print(lspeed);Serial.print(" "); Serial.print(rspeed); 
      //Serial.print("  out:"); Serial.println(out);
     lm = val+out;
     rm = val-out;
  } else {
     in2 = (double)lcnt - rcnt;
     pid2.Compute();
     lm = val + out2;
     rm = val - out2; 
     //Serial.print(lcnt); Serial.print("  "); Serial.println(rcnt);
  }
  
  if (dir == FORWARD){
      analogWrite(LF, lm); analogWrite(RF, rm); analogWrite(LR,0); analogWrite(RR,0);
  } else if (dir == BACKWARD){
      analogWrite(LF, 0); analogWrite(RF, 0); analogWrite(LR,lm); analogWrite(RR,rm);
  } else if (dir == LEFT){
      analogWrite(LF, 0); analogWrite(RF, rm); analogWrite(LR,lm); analogWrite(RR,0);
  } else if (dir == RIGHT){
      analogWrite(LF, lm); analogWrite(RF, 0); analogWrite(LR,0); analogWrite(RR,rm);
  } else if (dir == STOP){
      stop();
  }
  
  lcnt=0; rcnt=0;
}

void setup() {
  // put your setup code here, to run once:

  AlexDiagonal = sqrt (( ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH ));
  AlexCirc = PI * AlexDiagonal;
  
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();

  //PID
  MsTimer2::set(100, goPID);
  MsTimer2::start();
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-50,50);
  pid2.SetMode(AUTOMATIC);
  pid2.SetOutputLimits(-25,25);
  //left(0,130);
  //Color
  pinMode(CS_Out, INPUT);
  pinMode(CS_S0, OUTPUT);
  pinMode(CS_S1, OUTPUT);
  pinMode(CS_S2, OUTPUT);
  pinMode(CS_S3, OUTPUT);
  digitalWrite(CS_S0, HIGH);
  digitalWrite(CS_S1, LOW);
  //IR
  pinMode(FRONT_IR, INPUT);
  //reverse(0,130);
  //forward(0,130);
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
      break;
  }
}

void loop() {
  
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      }
  
  if(deltaDist > 0)
  {   
      if(dir==FORWARD)
      {    
          if(forwardDist > newDist || digitalRead(FRONT_IR) == 0)    
          {
              if(digitalRead(FRONT_IR) == 0) sendMessage("Front obstacle");
              deltaDist=0;
              newDist=0;
              stop();
          } else {
              //Serial.print("output: "); Serial.print(out); Serial.print("|"); 
              //Serial.print(lm); Serial.print(" "); Serial.println(rm);
          }
      }   
      else
          if(dir == BACKWARD)
          {     
              if(reverseDist > newDist)
              {
                  deltaDist=0;
                  newDist=0;
                  stop();
              } 
          }    
          else
              if(dir == STOP)
              {  
                  deltaDist=0;
                  newDist=0;
                  stop();
              } 
  }


//
if(deltaTicks > 0)
  {   
      if(dir==LEFT)
      {    
          if(leftReverseTicksTurns >= targetTicks)    
          {     
              deltaTicks=0;
              targetTicks=0;
              stop();
          } else {
          }
      }   
      else
          if(dir == RIGHT)
          {     
              if(rightReverseTicksTurns >= targetTicks)
              {
                  deltaTicks=0;
                  targetTicks=0;
                  stop();
              } 
          }    
          else
              if(dir == STOP)
              {  
                  deltaTicks=0;
                  targetTicks=0;
                  stop();
              } 
  }


}
