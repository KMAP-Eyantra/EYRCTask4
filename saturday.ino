//#include <Arduino.h>
//#include <SoftwareSerial.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#ifdef abs
#undef abs
#endif

#define abs(x) ((x)>0?(x):-(x))
#define PI 3.1415926535897932384626433832795
const int MPU = 0x68; // MPU6050 I2C address
//MPU6050 accelgyro;

volatile int tot_overflow;
volatile int timer1Flag = 0;
//const int pinA =  3; 

float init_time;
float dt2=20;
//#define rxPin 0
//#define txPin 1
float ax, ay, az;
float gx, gy, gz;
float yax, yay, yaz, ygx, ygy, ygz;
int f_cut = 5;
int n = 1;
float gxPrevious,gyPrevious,gzPrevious;
float ygx_prev, ygy_prev , ygz_prev , yax_prev ,yay_prev ,yaz_prev;
float roll,roll_prev,pitch,pitch_prev,yaw,omega;
float prev_lqr_torque,lqr_torque,torque;
float I_b = 0.0048167;
float x,x_dot,prev_x;
float theta,theta_dot,prev_theta;


//float k1=40.2370,  k2= 50.6292,k3=  -2000.5476,k4=   -50.7686;
//float k1=0.2370,  k2= -0.6292,  k3=  -900.00,  k4=   -0.0;
 //1.00000     0.51360  -105.28663    -7.34284
//float k1=-1.00000, k2=5.58,k3 = -15,k4= - 80.42088;
//float k1 = 31.623,   k2= 36.828, k3= -271.756,k4 =  -33.752;
//float k1 = 100.00, k2 =   109.09, k3= -1160.62,k4 =   -317.17;
//float k1 = 31.6228,k2=    12.8456,k3=  -134.2097,k4=    -8.2903;
//float k1 = 0.67316,k2=   -3.71111,k3=  -15.26510,k4=   -1.27307;
//float k1 = 0.086452,k2 = -1.228377,k3 =  -5.584156,k4=  -0.419026;
//float k1 = 0.25660,k2 = 1.66943,k3 = -6.05484,k4 = -0.55911;
//float k1 = 0.18126,k2 = -1.69480,k3 = -6.15988,k4 = -0.56176;
//float k1 =0.18126,k2 =-1.6948,k3 =-6.1599,k4 =-0.56176;
//float k1 =0.18287,k2 =-1.6138,k3 =-4.7406,k4 =-0.52045;
//float k1 =0.25887,k2 =-1.5836,k3 =-4.6478,k4 =-0.51804;
//float k1 =0.27322,k2 =0.1746,k3 =-4.2593,k4 =-0.00151;
//float k1 =0.27322,k2 =1.1746,k3 =-4.2593,k4 =-0.40151;
//float k1 =0.26943,k2 =1.2607,k3 =-5.3147,k4 =-0.43766;
//float k1 =1,k2 =0.1129,k3 =-65,k4 =-1.5;
//float k1 =1.5478,k2 =5.5149,k3 =-50.4391,k4 =-1.628;
//float k1 =1.9288,k2 =9.1311,k3 =-62.5589,k4 =-1.8164;
//float k1 =2.1986,k2 =7.9812,k3 =-51.3101,k4 =-3.5164;
//float k1 =2.4516,k2 =6.9707,k3 =-57.7848,k4 =-4.0715;
//float k1 =1.1575,k2 =1.2863,k3 =-61.005,k4 =-3.3925;
//float k1 =0.40436,k2 =-8.3847,k3 =-25.2536,k4 =-9.148;
//float k1 =1.1414,k2 =2.3377,k3 =-83.6186,k4 =-3.6214;
//float k1 =1.1881,k2 =-0.81173,k3 =-21.6576,k4 =-2.9583;
//float k1 =0.86509,k2 =-4.1194,k3 =-19.6959,k4 =-6.2396;
//float k1 =0.31623,k2 =0.079211,k3 =-5.8701,k4 =-0.48543;
//float k1 =1,k2 =0.57943,k3 =-6.8103,k4 =-1.0958;
float k1 =1.934,k2 =-2.6039,k3 =-20.4057,k4 =-6.2427;
float prev_time; //= millis();
float radius = 0.065/2.0, oneRevTicks = 270.0;
#define OUTPUT_READABLE_ACCELGYRO

//SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);


/*Team ID: 336
 * Team Members: Manish Dsilva, Amogh Zare, Pritam Mane, Kimaya Desai
 * 
 */
 //Macros
#define MagF            50                     // electromagnet pin
#define buzz_pin        31                     // Buzzer pin
#define RED_pin         43                     //LED Pin
#define Common_pin      45                     //LED Pin
#define GREEN_pin       47                     //LED Pin
#define BLUE_pin        49                     //LED Pin
#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  
#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

//Global Variables
int received;
int analogX;
int analogY;

#define encodPinR1      2                       // encoder A pin
#define encodPinR2      3                       // encoder B pin
#define encodPinL1      18                       // encoder A pin
#define encodPinL2      19                       // encoder B pin

#define LOOPTIME        100                     // PID loop time
#define FORWARD         1                       // direction of rotation
#define BACKWARD        2                       // direction of rotation
#define RIGHTWARD       3                       // direction of rotation
#define LEFTWARD        4                       // direction of rotation

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
volatile int count_r = 0;                                 // right rotation counter
volatile int countInit_r;
volatile int count_l = 0;                                 // left rotation counter
volatile int countInit_l;
volatile int tickNumber = 0;
volatile boolean run_r = false;                                     // motor moves
volatile boolean run_l = false; 


/*
 * motor_init(): To initialize motor pins
 * MAG_init(): To initialize Electromagnet Pins
 * LED_init(): To initialize the LED pins
 * BUZZ_init(): Buzzer initialization
 */
void setup() {
 Serial.begin(9600);
 motor_init();
 MAG_init(); 
 LED_init();
 BUZZ_init();
 accel_init();
 timer1_init();
 
 //pinMode(rxPin, INPUT);
 //pinMode(txPin, OUTPUT);
 //xbee.begin(19200);
 
}

void timer1_init()
{
    // set up timer with prescaler = 8
    TCCR1B |= (1 << CS11);
    //TCNT1 = 45536; //10ms
   // TCNT1 = 51536; //7ms
     TCNT1 = 57536; //4ms
    // TCNT1H = 0xC9;
    // TCNT1L = 0x50;
    // enable overflow interrupt
    TIMSK1 |= (1 << TOIE1);
    sei();
    tot_overflow = 0;
}

/*
 * Input: None
 * Output: None
 * Description: accelerometer Initialization
 * parameters: None
 */
void accel_init()
{
    //#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //Wire.begin();
    //#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //    Fastwire::setup(400, true);
    //#endif
   // Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  //  Wire.write(0x6B);                  // Talk to the register 6B
   // Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    //Wire.write(0x1F);                  //register for gy;
    //Wire.write(0x05); 
   // Wire.endTransmission(true);        //end the transmission

   // Wire.setClock(400000UL); 
   // delay(100);
  
    
    // initialize device
    //Serial.println("Initializing I2C devices...");
    //accelgyro.initialize();

    Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  //Wire.write(0x1F);                  //register for gy;
  //Wire.write(0x05); 
  Wire.endTransmission(true);        //end the transmission
    
}

void LED_init(){
    pinMode(RED_pin, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(GREEN_pin, OUTPUT);
    pinMode(BLUE_pin, OUTPUT);

    digitalWrite(RED_pin, HIGH);
    digitalWrite(Common_pin, HIGH);
    digitalWrite(GREEN_pin, HIGH);
    digitalWrite(BLUE_pin, HIGH);
}

/*
 * Input: None
 * Output: None
 * Description: Buzzer Initialization
 * parameters: None
 */
void BUZZ_init(){
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}

/*
 * Input: None
 * Output: None
 * Description: Motor Initialization
 * parameters: None
 */
void motor_init(void)
{
 pinMode(InR1, OUTPUT);
 pinMode(InR2, OUTPUT);
 pinMode(PWMR, OUTPUT);
 pinMode(InL1, OUTPUT);
 pinMode(InL2, OUTPUT);
 pinMode(PWML, OUTPUT);
 
 pinMode(encodPinR1, INPUT);
 pinMode(encodPinR2, INPUT);
 pinMode(encodPinL1, INPUT);
 pinMode(encodPinL2, INPUT);
 
 //digitalWrite(encodPinR1, HIGH);                      // turn on pullup resistor
 //digitalWrite(encodPinR2, HIGH);
 //digitalWrite(encodPinL1, HIGH);                      // turn on pullup resistor
 //digitalWrite(encodPinL2, HIGH);
 
 //attachInterrupt(1, rencoder, RISING);               // arduino pin 3
 //attachInterrupt(4, lencoder, RISING);               // arduino pin 21
}

void moveMotor(int direction, int PWM_val, long tick)  
{
 count_r = 0;
 count_l = 0;
 countInit_r = count_r;    // abs(count)
 countInit_l = count_l;
 tickNumber = tick;
 if(direction==FORWARD)          
 {  
  motorForward_R(PWM_val);
  motorForward_L(PWM_val);
 }
 else if(direction==BACKWARD)    
 { 
  //Serial.println("Backward");
  motorBackward_R(PWM_val);
  motorBackward_L(PWM_val);
 }
 else if(direction==RIGHTWARD)          
 {  
  motorForward_R(PWM_val);
  motorBackward_L(PWM_val);
 }
 else if(direction==LEFTWARD)          
 {  
  motorBackward_R(PWM_val);
  motorForward_L(PWM_val);
 }
 
}

void rencoder()  
{                                    // pulse and direction, direct port reading to save cycles              
 if(digitalRead(encodPinR1)==HIGH)   count_r ++;  //motors forward ticks++              
 else if (digitalRead(encodPinR2)==HIGH)   count_r --; //motors backward ticks--   
 if(run_r) 
   if((abs(abs(count_r)-abs(countInit_r))) >= tickNumber)      motorBrake_R();
}

void lencoder()  
{                                    // pulse and direction, direct port reading to save cycles              
 if(digitalRead(encodPinL1)==HIGH)   count_l ++;  //motors forward ticks++              
 else if (digitalRead(encodPinL2)==HIGH)   count_l --; //motors backward ticks--   
 if(run_l) 
   if((abs(abs(count_l)-abs(countInit_l))) >= tickNumber)      motorBrake_L();
}


void motorForward_L(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}

/*
 * Input: None
 * Output: None
 * Description: Forward Motion of Right Motor
 * parameters: PWM value
 */
void motorForward_R(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
  
}


/*
 * Input: None
 * Output: None
 * Description: Backward Motion of Left Motor
 * parameters: PWM value
 */
void motorBackward_L(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}


/*
 * Input: None
 * Output: None
 * Description: Backward Motion of Right Motor
 * parameters: PWM value
 */
void motorBackward_R(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}


void motorBrake()  {
 analogWrite(PWMR, 0);
 analogWrite(PWML, 0);
 digitalWrite(InR1, HIGH);
 digitalWrite(InR2, HIGH);
 digitalWrite(InL1, HIGH);
 digitalWrite(InL2, HIGH);
 run_r = false;
 run_l = false;
}

void motorBrake_R()  
{
 analogWrite(PWMR, 0);
 digitalWrite(InR1, HIGH);
 digitalWrite(InR2, HIGH);
 run_r = false;
}

void motorBrake_L()  
{
 analogWrite(PWML, 0);
 digitalWrite(InL1, HIGH);
 digitalWrite(InL2, HIGH);
 run_l = false;
}

/*
 * Input: None
 * Output: None
 * Description: Electromagnet Initialization
 * parameters: None
 */
void MAG_init(void)
{
    pinMode(MagF, OUTPUT);
    digitalWrite(MagF, LOW);
}

/*
 * Input: None
 * Output: None
 * Description: Magnet Pickup
 * parameters: None
 */
void MagPick(void)  
{
    digitalWrite(MagF, HIGH);
}

/*
 * Input: None
 * Output: None
 * Description: Magnet Drop
 * parameters: None
 */
void MagDrop(void)  
{
    digitalWrite(MagF, LOW);
}


void lowpassfilter(float ax,float ay,float az,int16_t n,int16_t f_cut)
{
  float dT = 0.004;  //time in seconds
  float Tau= 1/(2*3.1457*f_cut);                   //f_cut = 5
  float alpha = Tau/(Tau+dT);                //do not change this line

  if(n == 1)
  {
    yax = (1-alpha)*ax ;
    yay = (1-alpha)*ay ;
    yaz = (1-alpha)*az ;
  }
  else
  {
    yax = (1-alpha)*ax + alpha*yax_prev;
    yay = (1-alpha)*ay + alpha*yay_prev;
    yaz = (1-alpha)*az + alpha*yaz_prev;  
  }  
  yax_prev = yax;
  yay_prev = yay;
  yaz_prev = yaz;
    
}

void highpassfilter(float gx,float gy,float gz,int16_t n,int16_t f_cut)
{
  
  float dT = 0.004;  //time in seconds
  float Tau= 1/(2*3.1457*f_cut);                   //f_cut = 5
  float alpha = Tau/(Tau+dT);                //do not change this line
 
  if(n == 1)
  {
    ygx = (1-alpha)*gx ;
    ygy = (1-alpha)*gy ;
    ygz = (1-alpha)*gz ;

  }
  else
  {
    ygx = (1-alpha)*ygx_prev + (1-alpha)*(gx - gxPrevious);
    ygy = (1-alpha)*ygy_prev + (1-alpha)*(gy - gyPrevious);
    ygz = (1-alpha)*ygz_prev + (1-alpha)*(gz - gzPrevious);
  }
  gxPrevious = gx;
  gyPrevious = gy;
  gzPrevious = gz;

  ygx_prev = ygx;
  ygy_prev = ygy;
  ygz_prev = ygz;
  
}

void comp_filter_pitch(float ax,float ay,float az,float gx,float gy,float gz)
{
  float alpha = 0.03;
  float dt = 0.007;

  if (n==1)
  {
    pitch = (1-alpha)*((-1)*gx*dt) + alpha*(atan(ay/abs(az))*180/PI); 
  }
  else
  {
    pitch = (1-alpha)*(pitch_prev - (gx*dt)) + alpha*(atan(ay/abs(az))*180/PI);
  }
  pitch_prev=pitch;
}

void comp_filter_roll(float ax,float ay,float az,float gx,float gy,float gz)
{
  float alpha = 0.01;
  float dt = 0.004;

  if (n==1)
  {
    roll = (1-alpha)*((-1)*gy*dt) + alpha*(atan2(ax,abs(az))*180/PI);
  }
  else
  {
    roll = (1-alpha)*(roll_prev - (gy*dt)) + alpha*(atan2(ax,abs(az))*180/PI);
  }
  roll_prev=roll;  
}
void motorControl(int torque)
{
 //torque between 0-255
 if (torque >= 0) 
 { // drive motors forward

 torque = abs(torque);
 if(torque<60)
  torque = 60;
 motorForward_R(torque); 
 motorForward_L(torque); 
}
 else{ 
 // drive motors backward

 torque = abs(torque);
 if(torque<60)
  torque = 60;
 motorBackward_R(torque); 
 motorBackward_L(torque);
 }
}

ISR(TIMER1_OVF_vect)
{
 // TCNT1 = 45536;  //10ms
 // TCNT1 = 51536; //7ms
  TCNT1 = 57536; //4ms
 // TCNT1H = 0xC9;
 // TCNT1L = 0x50;
  timer1Flag=1;
   //tot_overflow++;
  

    /*if (tot_overflow >= 1000 ) // NOTE: '>=' used instead of '=='
    {
        //flag=1;
        digitalWrite(3,LOW);
        //tot_overflow = 0;   // reset overflow counter
    }*/
}

void  readTiltAngle()
{ 
  //MPU
  //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelGyro();
  //Serial.println(accelgyro.getRate());
  //lowpassfilter(ax,ay,az,n,2);
  //highpassfilter(gx,gy,gz,n,2);
  //comp_filter_pitch(yax,yay,yaz,ygx,ygy,ygz);
  //comp_filter_roll(yax,yay,yaz,ygx,ygy,ygz);
  comp_filter_roll(ax,ay,az,gx,gy,gz);
  //MPU ENDS
  
}

void lqrControl()
{
  //STATE VARIABLES
  
theta = (roll);//-0.09);
  //theta_dot = (theta-prev_theta)/dt2;
  theta_dot = (theta-prev_theta);
  if(n%5 == 0)
    {
      x = ((count_r)*2*PI*radius)/270.0; //displacement
      x_dot = (x - prev_x);
      countInit_r = count_r;
    }
  //STATE VARIABLES ENDS
  //Serial.println(theta);
  
  
  //PID
  int pid_torque = 300*theta+900*(theta-prev_theta)- 0*x-0*(x-prev_x);
  pid_torque = constrain(pid_torque, -150, 150);
  //if(pid_torque>=0)
  //pid_torque = map(pid_torque, 0, 200, 60,200);
  //else 
  //pid_torque = map(pid_torque, -200, 0,-200,-60);
  
  //PID ENDS

  //Serial.print(300*theta);Serial.print("\t");Serial.println(250*(theta-prev_theta));
  lqr_torque =  ((-x*k1)+(-x_dot*k2)+(theta*k3/57.0)+(theta_dot*k4/57.0))*(255.0/12.0);
  lqr_torque = 14.400921*lqr_torque; 
  lqr_torque = constrain(lqr_torque, -200, 200); //
  //Serial.print(x);Serial.print("\t");Serial.println(theta);
  
  //Serial.print(x*k1);Serial.print("\t");Serial.print(x_dot*k2);Serial.print("\t");Serial.print(theta*k3/57.0);Serial.print("\t");Serial.println(-theta_dot*k4/57.0);
  //PREVIOUS STATE VARIABLES
  prev_theta = theta;
  prev_x = x;
  //PREVIOUS STATE VARIABLES ENDS
  
  //Serial.print(abs(lqr_torque)); Serial.println();
  //Serial.println(constrain(pid_torque,-100,100));

  //Serial.println(theta);
  //Serial.println(theta_dot);
  
  //prev_lqr_torque = lqr_torque;
  //Serial.println(x_dot);
  //if(n%1 == 0)
  motorControl(lqr_torque);
  /*
  else if(yax>yaxUpperThreshold)
  {
    //Serial.print("Falling Backward");
    motorControl(-abs(lqr_torque));
  }
  else if(yax<yaxLowerThreshold)
  {
    //Serial.print("Falling Forward");Serial.print("\t");
    motorControl(abs(lqr_torque));
  }
  */
}

void zigbeeControl()
{
  
  //Serial.println();
  
  //Accept only if characters are 18 or more
  
  if(Serial.available()>=18)                                
  {
    //0x7E is the first byte of Xbee frame
    //Accept data starting with 0X7E
    if(Serial.read()==0x7E)
    {
      
      //Ignore the first 11 bytes of data
      for(int i=1 ; i < 11 ; i++)
      {
        byte discardByte = Serial.read();    
      }

      //Accept the data from the 11th byte to 16th byte
      byte digitalMSB = Serial.read();
      byte digitalLSB = Serial.read();
      byte analogMSB1 = Serial.read();
      byte analogLSB1 = Serial.read();
      byte analogMSB2 = Serial.read();
      byte analogLSB2 = Serial.read();

      //Combining LSB and MSB by left shifting the MSB and adding it to the LSB
      int analogX = analogLSB1+ analogMSB1*256;
      int analogY = analogLSB2+ analogMSB2*256;
      
      //Serial.print(analogX);
      //Serial.print("\t");
      //Serial.println(analogY);

      int else_flag = 0;

      //Turn On the LED if the digitalMSB value is 0x01
      if(digitalMSB == 0x01) //led switch
      {
        digitalWrite(RED_pin, LOW);
        digitalWrite(GREEN_pin, HIGH); 
        digitalWrite(BLUE_pin, HIGH); 
        digitalWrite(MagF, LOW);
        digitalWrite(buzz_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the Magnet if the digitalLSB value is 0x04
      if(digitalLSB == 0x04) //magnet switch
      {
        digitalWrite(MagF, HIGH);
        digitalWrite(buzz_pin, HIGH);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the Buzzer if the digitalLSB value is 0x08
      else if(digitalLSB == 0x08) //buzzer switch
      {
        digitalWrite(buzz_pin, LOW);
        digitalWrite(MagF, LOW);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the Magnet and Buzzer if the digitalLSB value is 0x0C
      else if(digitalLSB == 0x0C) //magnet and buzzer both
      {
        digitalWrite(MagF, HIGH);
        digitalWrite(buzz_pin, LOW);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn off everything
      else if(else_flag == 0)
      {
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);    
        digitalWrite(MagF, LOW);
        digitalWrite(buzz_pin, HIGH);
      }
      
      //No Motion
      //Serial.print(analogX);Serial.print("\t");Serial.println(analogY);
      if((analogX > 1023) || (analogY > 1023 ))
        int a;
  
      //Both Wheels Forward
      else if((analogX > 900)  && (400 < analogY) && (analogY <800 ))
      {
        Serial.println("Both Wheels Forward");
        moveMotor(FORWARD,  200, 3*2);
      }
  
      //Both Wheels Backward
      else if((analogX > 900) && (analogY < 300) )
      {
        Serial.print("LSB ");
        Serial.println(digitalLSB);
        Serial.println(digitalMSB);
        moveMotor(BACKWARD,  200, 3*2);
      }
  
      //Left Wheel Backward, Right Wheel Forward
      else if((400< analogX)&&(analogX <800 )&& (analogY > 900) )
      {
        Serial.println("left wheel Backward, right wheel Forward");
        moveMotor(RIGHTWARD,  200, 3*2);
      }
  
      //Left Wheel Forward, Right Wheel Backward
      else if((analogX < 300) && (analogY > 900))
      {
        Serial.println("left wheel Forward, right wheel Backward");
        moveMotor(LEFTWARD,  200, 3*2);
      }

      else
      {
        motorBrake();
      }
    
    } 
  } 
}

void accelGyro()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  ax = (Wire.read() << 8 | Wire.read())/ 16384.0; // X-axis value
  ay = (Wire.read() << 8 | Wire.read())/ 16384.0; // Y-axis value
  az = (Wire.read() << 8 | Wire.read())/ 16384.0; // Z-axis value

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 

  gx = (Wire.read() << 8 | Wire.read()) ; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gy = (Wire.read() << 8 | Wire.read())/131.0 ;
  gz = (Wire.read() << 8 | Wire.read()) ;
gy-=5.35;//*131;//*2.28;

ax=ax-0.03;
ay=ay+0.005+0.02;
az=az-0.08+0.02;
//   
//  Serial.print("\tGyX: ");
//  Serial.print(gx);
//  Serial.print("\tGyY: ");
//  Serial.print(gy);
////  Serial.print("\tGyZ: ");
////  Serial.print(gz);
//
//  Serial.print("\taX: ");
//  Serial.print(ax);
//  Serial.print("\taY: ");
//  Serial.print(ay);
//  Serial.print("\taZ: ");
//  Serial.println(az);
//  

}


void loop()
{
  if(timer1Flag==1)
  {
    readTiltAngle(); 
    lqrControl();
   // zigbeeControl();
    timer1Flag=0;
  }
  Serial.println(roll);
  //Serial.println(dt2*1000);
  //delay(7); 
  n++;
  
}
