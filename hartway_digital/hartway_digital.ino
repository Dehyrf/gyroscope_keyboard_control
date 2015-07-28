#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

int count123 = 0;

uint8_t buf[8] = { 
  0 };   /* Keyboard report buffer */
 
//#define Wkey 5
//#define Akey 6
//#define Skey 7
//#define Dkey 8
 
int state123 = 1;

//Variables for GYRO_GAIN and ACCEL_GAIN
#define ACCEL_GAIN 18.0 //
//#define ACCEL_GAIN 17.0 //
//#define GYRO_GAIN 6.0   //
#define GYRO_GAIN 5.0   //

float ti_constant = 3;

const float ANGLE_GAIN = 1.20; //20% increase in angle measurement.
//const float ANGLE_GAIN = 1.15; //15% increase in angle measurement.

float aa_constant = 0.005; //this means 0.5% of the accelerometer reading is fed into angle of tilt calculation with every loop of program (to correct the gyro).
//accel is sensitive to vibration which is why we effectively average it over time in this manner. You can increase aa if you want to experiment. 
//too high though and the board may become too vibration sensitive.

//Debug  note: "cntrl /" for group comment
#define DEBUG_FORCE_DEADMAN_SWITCH 0 //normal
#define DEBUG_ENABLE_PRINTING 0 //normal
#define DEBUG_DISABLE_MOTORS 0 //normal

//#define DEBUG_FORCE_DEADMAN_SWITCH 1 //DEBUG ONLY...Force on for debug only.  Not for operation!!
//#define DEBUG_ENABLE_PRINTING 1 //DEBUG ONLY... turn off for real operation!
//#define DEBUG_DISABLE_MOTORS 1 //DEBUG ONLY... turn off for real operation!
//Debug 

//Note: Set Sabertooth dip switches on the board for simplified serial and 9600 Baudrate. 
#define SABER_TX_PIN  13 //Digital pin 13 is serial transmit pin to sabertooth
#define SABER_RX_PIN  12 //Not used but still initialised, Digital pin 12 is serial receive from Sabertooth
#define SABER_BAUDRATE  9600 //set baudrate to match sabertooth dip settings

//simplifierd serial limits for each motor
#define SABER_MOTOR1_FULL_FORWARD 1
#define SABER_MOTOR1_FULL_REVERSE 127
#define SABER_MOTOR2_FULL_FORWARD 128
#define SABER_MOTOR2_FULL_REVERSE 255


#define SABER_ALL_STOP  0 //motor command to send when issuing full stop command

//SoftwareSerial SaberSerial = SoftwareSerial (SABER_RX_PIN, SABER_TX_PIN );
                                             
/*void initSabertooth (void)  { //initialize software to communicate with sabertooth 
  pinMode ( SABER_TX_PIN, OUTPUT );
  SaberSerial.begin( SABER_BAUDRATE );
  SaberSerial.write((byte) 0);   //kill motors when first switched on
}
*/
//note MPU6050 connections: 
//SCL = A5
//SDA = A4
//INT   = Digital 2 on arduino Uno
//Vcc  = 5V
//Gnd  = Gnd

#define MPU_INT 0 //is on pin 2

MPU6050 mpu;   // AD0 low = 0x68

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float angle_Y, angular_rate_Y, angular_rate_X;
float angle_X, angle_Z, angular_rate_Z;
bool blinkState = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

// Aruino Pin Assignments:
//int deadmanButtonPin 		= 9;  // deadman button is digital input pin 
//int balanceForwardPin 	        = 5;  //if digital pin  is 5V then reduce balancepoint variable. Allows manual fine tune of the ideal target balance point
//int balanceBackwardPin 	        = 8;  //if digital pin  is 5V then increase balancepoint variable. Allows manual fine tune of the ideal target balance point
//int steeringLeftPin 	        = 7;  //digital pin Used to steer
//int steeringRightPin 	        = 4;  //digital pin Used to steer the other way.

//int greenLedPin 		= 10; // bi-color LED connected to digital pin 
//int commonHighLedPin 	        = 11; // bi-color LED connected to digital pin 
//int redLedPin 			= 12; // bi-color LED connected to digital pin 

//reserved pin  		= 2;  //accel/gyro IMU interrupt 0 pin input
//reserved pin 			= 13  //output to saber serial motor controller 
//reserved pin                  = A4 //MPU6050 SDA
//reserved pin                  = A5 //MPU6050 SCL
//int spare 			= 6; //spare pin
int oscopePin 			= 3; //spare pin or to oscope to see cycle time for debug


float cur_speed;
float cycle_time = 0.01; //seconds per cycle - currently 10 milliseconds per loop of the program.  
                         // Need to know it as gyro measures rate of turning. Needs to know time between each measurement
                         //so it can then work out angle it has turned through since the last measurement - so it can know angle of tilt from vertical.

int STD_LOOP_TIME = 9; //9= 10mS loop time // code that keeps loop time at 10ms per cycle of main program loop 
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

float level=0;
//float Steering;
//float SteerValue;
//float SteerCorrect;
//int   Steer = 0;

float x_acc;
float SG_filter_result;
float x_accdeg;

float initial_angular_rate_Y = 0;
float initial_angular_rate_Y_sum = 0;
float initial_angular_rate_X = 0;
float initial_angular_rate_X_sum = 0;

//float gangleratedeg;
//float gangleratedeg2;

//float gangleraterads;
//int   SteerLeftPin;
//int   SteerRightPin;
//int   DeadManPin;

//add for deadman debounce
//int DeadManPin_temp = 1; //this variable is from the digitalRead
//int DeadManPin_temp_old = 1; //this variable is delayed from the digitalRead
//long lastDebounceTime = 0;  // the last time the output pin was toggled
//long debounceDelay = 50;    // the debounce delay in mSecs

float overallgain; 

float gyroangle_dt;
float angle;
float anglerads;
float balance_torque;
float softstart;

float Balance_point;
float balancetrim = 0;

int balancelForward;
int balancelBackward;

float gv0, gv1, gv2, gv3, gv4, gv5, gv6;  //Sav Golay variables.  filter for accelerometer called Savitsky Golay filter.

int i;
int j;
int tipstart;

//signed char Motor1percent;
//signed char Motor2percent;

//int deadman_occured_flag;
//int skip = 0;//for debug


////////////////////////////////////////////////////////////////////////////////
void setup() { // run once, when the sketch starts
  ////////////////////////////////////////////////////////////////////////////////
 
 // initSabertooth(); //initialze saber motor controller
  //SaberSerial.write((byte) 0);   //kill motors when first switched on

  Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
   
  Serial.begin(115200); // initialize I2C and serial monitor to 115,200 baud
   
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(2);
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(10);
  mpu.setYGyroOffset(7);
  mpu.setZGyroOffset(14);
  mpu.setZAccelOffset(900); // 1688 factory default for  test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
    {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
    
      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(MPU_INT, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
    
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
    
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
  else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }

  //Initialize IO pins

  //Init LED
  //  pinMode(redLedPin, OUTPUT);      // sets the digital pin as output
   // pinMode(commonHighLedPin, OUTPUT);      // sets the digital pin as output
    //pinMode(greenLedPin, OUTPUT);      // sets the digital pin as output
    //digitalWrite(redLedPin, LOW);   //LED defaults to RED at setup
    //digitalWrite(commonHighLedPin, HIGH);   //LED common is high.
    //digitalWrite(greenLedPin, HIGH);   //LED 

  //digital inputs
    //pinMode(deadmanButtonPin, INPUT);
    //digitalWrite(deadmanButtonPin, HIGH);                     // turn on pullup resistors
    //pinMode(balanceForwardPin, INPUT);
    //digitalWrite(balanceForwardPin, HIGH);                  // turn on pullup resistors
    //pinMode(balanceBackwardPin, INPUT);
    //digitalWrite(balanceBackwardPin, HIGH);                 // turn on pullup resistors
    //pinMode(steeringLeftPin, INPUT);
    //digitalWrite(steeringLeftPin, HIGH);              // turn on pullup resistors
    //pinMode(steeringRightPin, INPUT);
    //digitalWrite(steeringRightPin, HIGH);             // turn on pullup resistors
  
  //init oscope output
    pinMode(oscopePin, OUTPUT);      // sets the digital pin as output
    digitalWrite(oscopePin, LOW);   //
  
  
    //Delay 2 seconds to let MPU6050 self calibrate before reading gyros
    delay (2000); // 2 seconds

    // At start of loop, read the accel/gyro multiple times to get an average baseline value.  
    // This will be subtracted from the current value in the balance loop.
    for (j=0; j<7; j++) {
      read_accel_gyro();
      initial_angular_rate_Y_sum = (float) initial_angular_rate_Y_sum  + angular_rate_Y; //sum of the 7 readings of front/back tilt gyro
      initial_angular_rate_X_sum = (float) initial_angular_rate_X_sum  + angular_rate_X; //sum of the 7 readings left/right steer gyro
      //delay to do accel/gyro reads.
      delay (10); //10ms
    }
    initial_angular_rate_Y = (float) initial_angular_rate_Y_sum/7;  //initial front/back tilt gyro
    initial_angular_rate_X = (float) initial_angular_rate_X_sum/7;  //initial left/right steer gyro

}//end of setup

//////////////////////////////////////////////////////////////////////////////////////////

   
////////////////////////////////////////////////////////////////////////////////
void loop ()   {
////////////////////////////////////////////////////////////////////////////////

  tipstart = 0;
  overallgain = 0;
  cur_speed = 0;
  level = 0;
  //Steer = 0;
  angle = 0;
  //Steering = 512;
  //SteerValue = 512;
    
  overallgain = 0.3;  //softstart value. Gain will now rise to final of 0.5 at rate of 0.005 per program loop. 
  //i.e. it will go from 0.3 to 0.5 over the first 4 seconds after tipstart has been activated

  //After this point the machine is active.
  //Main balance routine, just loops forever. Machine is just trying to stay level. You "trick" it into moving by tilting one end down
  //works best if keep legs stiff so you are more rigid like a broom handle is if you are balancing it vertically on end of your finger
  //if you are all wobbly, the board will go crazy trying to correct your own flexibility.
  while (1) {
     
    read_accel_gyro(); // read accel/gyro
    type_keyboard();
  
    //XXXXXXXXXXXXXXXXXXXXX loop timing control keeps it at 100 cycles per second XXXXXXXXXXXXXXX
    lastLoopUsefulTime = millis() - loopStartTime;
   
    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      delay(STD_LOOP_TIME - lastLoopUsefulTime);
    }
    
    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();   
    //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX   

   // serialOut_timing();//for debug only, displays loop time on screen 
                       // first digit is time loop takes to run in millisec, 
                       // second digit is final time for loop including the variable added delay to keep it at 100Hz
  
    //XXXXXXXXXXXXXXXXXXXX softstart function: board a bit squishy when you first bring it to balanced point, then ride becomes firmer over next 4 seconds XXXXXXXXXXXXXXX  
    if (overallgain < 0.5) {
      overallgain = (float)overallgain + 0.005;
    }
    if (overallgain > 0.5) {
      overallgain = 0.5;
    }
    //XXXXXXXXXXXXXXX end of softstart code XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  
    //   //send out oscope debug pulse
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, HIGH);   
    //   digitalWrite(oscopePin, LOW);   
  
  }  //end of while(1)       
} //end of main LOOP
  
  
  
////////////////////////////////////////////////////
// functions start here
///////////////////////////////////////////////////
void type_keyboard()  {
if (angle_Y >= 25) {
    buf[2] = 26;   // w
    Serial.write(buf, 8); // Send keypress
    releaseKey();
  } 
  if (angle_Y <= -25) {
    buf[2] = 22;   // w
    Serial.write(buf, 8); // Send keypress
    releaseKey();
  } 
if (angle_Z >= 25) {
    buf[2] = 4;   // w
    Serial.write(buf, 8); // Send keypress
    releaseKey();
  } 
  if (angle_Z <= -25) {
    buf[2] = 7;   // w
    Serial.write(buf, 8); // Send keypress
    releaseKey();
  } 






  
}
void releaseKey() 
{
  buf[0] = 0;
  buf[2] = 0;
  Serial.write(buf, 8); // Release key  
}

////////////////////////////////////////////////////////////////////////////////
void read_accel_gyro()  {     //digital accel/gyro is read here
  ////////////////////////////////////////////////////////////////////////////////
  
  if (!dmpReady) return; // if programming failed, don't try to do anything
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //Serial.print(" fifoCount: ");
      //Serial.print(fifoCount);
      //Serial.print(" mpuIntStatus: ");
      //Serial.print(mpuIntStatus);
      //Serial.println(F("FIFO overflow!"));
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
  else if (mpuIntStatus & 0x02)
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      //Get sensor data
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGyro(gyro, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
      // angle and angular rate
      angle_X = ypr[0]* RAD_TO_DEG;             // not used...0 is center of gravity offset
      angle_Y = ypr[1]* RAD_TO_DEG;             // Accel for Tilt, 0 is center of gravity offset
      angle_Z = ypr[2]* RAD_TO_DEG;             // not used...0 is center of gravity offset
      angular_rate_X = ((double)gyro[0]/131.0); // Gyro for steering, in degs/sec.
      angular_rate_Y = ((double)gyro[1]/131.0); // Gyro for tilt, in degs/sec.
      angular_rate_Z = ((double)gyro[2]/131.0); // Gyro for X, in degs/sec.
     
      angular_rate_X = angular_rate_X * RAD_TO_DEG; // Gyro for steering, in degs/sec.
      angular_rate_Y = angular_rate_Y * RAD_TO_DEG; // Gyro for tilt, 
      angular_rate_Z = angular_rate_Z * RAD_TO_DEG; // Gyro for X
    
    } //end else if (mpuIntStatus & 0x02)
}//end of read_accel_gyro()  
