#include "TimerOne.h"

#define M1PWM 6//11//10
#define M2PWM 5//10//11
#define M1F 13//9
#define M2F 11//9//13
#define M1B 12//8
#define M2B 10//8//12
#define M1_encoderA 2
#define M2_encoderA 3
#define M1_encoderB 8//4
#define M2_encoderB 9//5

// Motor 1
volatile unsigned int position1=0;
volatile int speed1=0;
volatile unsigned int old_position1=0;
volatile int old_speed1=0;
// Motor 2
volatile unsigned int position2=0;
volatile int speed2=0;
volatile unsigned int old_position2=0;
volatile int old_speed2=0;

int flag=0;
unsigned long start_T=0;
unsigned long lastT=0;
int LOOP=100;
int lastOutput=0;

int i=0;
int d=0; //direction

int CPR=32;
int V1,V2;
int count=0;

int MotorV=0;
int MotorV1=0;
int MotorV2=0;
int mode=0;
int flexLabelId =0;
int flexLabelId1=0;
int flexLabelId2=0;
int flexLabelId3=0;
int direct=0;
int Stop=0;

int r = 75;
int g = 75;
int b = 110;

int Setpoint1, Input1, Output1, Setpoint2, Input2, Output2;
//Define the aggressive and conservative Tuning Parameters
double aggKp=0.8, aggKi=0.2, aggKd=0.1;
double consKp=0.5, consKi=0.2, consKd=0.05;
double Kp, Ki, Kd;
double error1,error2,last_error1,last_error2;

String str;


void setup() 
{ 
  pinMode(M1PWM, OUTPUT);
  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  digitalWrite(M1F, HIGH);
  digitalWrite(M1B,HIGH);
  
  pinMode(M2PWM, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
  digitalWrite(M2F, HIGH);
  digitalWrite(M2B,HIGH);
  
  pinMode(M1_encoderA, INPUT);
  digitalWrite(M1_encoderA,HIGH);
  pinMode(M1_encoderB, INPUT);
  digitalWrite(M1_encoderB,HIGH);
  
  pinMode(M2_encoderA, INPUT);
  digitalWrite(M2_encoderA,HIGH);
  pinMode(M2_encoderB, INPUT);
  digitalWrite(M2_encoderB,HIGH);
  
  digitalWrite(M1F, HIGH);
  digitalWrite(M1B,HIGH);
  
  digitalWrite(M2F, HIGH);
  digitalWrite(M2B,HIGH);
  Input1=0;
  Setpoint1=0;
  Output1=0;
  error1=0;
  last_error1=0;
  
  Input2=0;
  Setpoint2=0;
  Output2=0;
  error2=0;
  last_error2=0;
    
  
  attachInterrupt(0, doEncoder1, CHANGE);
  attachInterrupt(1, doEncoder2, CHANGE);
  //Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 20Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  //Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  
  Direct1(0);
  Direct2(0);
  //gBegin(34236);
  Serial.begin(115200);
//  Serial.begin(115200);
  while (! Serial);
  
  
} 

void loop() 
{ 
 //Serial.println("Ready"); 
 if(Serial.available())
 {  
      //Serial.println("Pi");
      //guino_update();
      direct=(Serial.read()-'0');
      Serial.flush();
 }
     getParams(direct);
      count++;
     
     if(millis()-lastT>=LOOP)
     {
        
       
         timerIsr();
         lastT=millis();
      
      V1=(V1*60000/LOOP)/CPR/131;
      V2=(V2*60000/LOOP)/CPR/131;
      Input1=V1;
      Input2=V2;
      //error1 = abs(Setpoint1-Input1);
      //error2 = abs(Setpoint2-Input2);
      error1 = abs(Setpoint1)-abs(Input1);
      error2 = abs(Setpoint2)-abs(Input2);
      
      AdaptPara(abs(error1));
      Output1=PID(Output1,error1,last_error1,LOOP/1000.0);
      last_error1=error1;
      //error1=0;
      AdaptPara(abs(error2));
      Output2=PID(Output2,error2,last_error2,LOOP/1000.0);  
      last_error2=error2;
      //error2=0;
      //AdaptPara(error1);
      //Output1=PID(Output1, Setpoint1, Input1);
      
      //AdaptPara(error2);
      //Output2=PID(Output2, Setpoint2, Input2); 
      //str=String(Input1); 
      //Serial.println(Input1,DEC);
      
      
     // Serial.println(Output);
      analogWrite(M1PWM, Output1);
      analogWrite(M2PWM, Output2);
     }
     
  //} 
     
     
     
      
}

void doEncoder1()
{
  //if(digitalRead(M1_encoderA)==HIGH)
  speed1++;
  if (digitalRead(M1_encoderA) == digitalRead(M1_encoderB))
    position1++;
  else
    position1--;
    
     
}

void doEncoder2()
{
  speed2++;
  if (digitalRead(M2_encoderA) ==digitalRead(M2_encoderB))
    position2++;
  else
    position2--;
       
}

void timerIsr()
{
  //V1=(speed1-old_speed1)*1000/LOOP/CPR;
  V1=speed1-old_speed1;
  old_speed1=speed1;
  
  V2=speed2-old_speed2;
  old_speed2=speed2;
    
}

void Direct1(int direction) // 0=forward; 1=back
{
  if (direction == 0)
  {
    digitalWrite(M1F, HIGH);
    digitalWrite(M1B,LOW);
  }
  else if (direction == 1)
  {  
    digitalWrite(M1F, LOW);
    digitalWrite(M1B,HIGH);
  }
  if (direction == 2)
  {
    digitalWrite(M1F, HIGH);
    digitalWrite(M1B,HIGH);
  }
}

void Direct2(int direction) // 0=forward; 1=back
{
  if (direction == 0)
  {
    digitalWrite(M2F, HIGH);
    digitalWrite(M2B,LOW);
  }
  else if (direction == 1)
  {  
    digitalWrite(M2F, LOW);
    digitalWrite(M2B,HIGH);
  }
  if (direction == 2)
  {
    digitalWrite(M2F, HIGH);
    digitalWrite(M2B,HIGH);
  }
}

/*int PID(int command, int targetValue, int currentValue)   {      // compute PWM value

float pidTerm = 0;                                                           // PID correction

int error=0;                                 

static int last_error=0;                            

  error = abs(targetValue) - abs(currentValue);

  pidTerm = (Kp * error) + (Kd * (error - last_error));                           

  last_error = error;

  return constrain(command + int(pidTerm), 0, 255);

}*/
int PID(double command,int error,int last_error, float time)   {      // compute PWM value

float pidTerm = 0;                                                           // PID correction

  pidTerm = (Kp * error) +(Ki*error*time)+ (Kd * (error - last_error));                           
  return constrain(command + int(pidTerm), 0, 255);

}

void AdaptPara(double error)
{
  if(error>2)
  {
  if(error<6)
      {  //we're close to setpoint, use conservative tuning parameters
            //myPID.SetTunings(consKp, consKi, consKd);
            Kp=consKp;
            Ki=consKi;
            Kd=consKd;
      }
      else
      {
      //we're far from setpoint, use aggressive tuning parameters
         //myPID.SetTunings(aggKp, aggKi, aggKd);
            Kp=aggKp;
            Ki=aggKi;
            Kd=aggKd;
      } 
  }
}

int getParams(int direct)
{

  //if(Serial.available()) 
 // {
    
    switch(direct)
    {
      case 0:
      // Stop
        flag=0;
        Direct1(2);
        Direct2(2);
        Setpoint1=0;
        Setpoint2=0;
        error1=0;
        error2=0;
       // Serial.println(direct);
        break;
      case 1:
      // Forward
        flag=0;
        Direct1(0);
        Direct2(0);
        Setpoint1=15;
        Setpoint2=15;
        //Serial.println(direct);
        break;
      case 2:
      // Backward
      
        if (flag ==0){
        Setpoint1=0;
        Setpoint2=0;
        flag = 1;
        }
        Direct1(1);
        Direct2(1);
        Setpoint1=15;
        Setpoint2=15;
        //Serial.println(direct);
        break;
      case 3:
      // Turn Right
        if (flag == 0){
        Setpoint1=0;
        Setpoint2=0;
        flag=1;
        }
        Direct1(1);
        Direct2(0);
        Setpoint1=15;
        Setpoint2=15;
        //Serial.println(direct);
        break;
      case 4:
      // Turn Left
        if (flag==0){
        Setpoint1=0;
        Setpoint2=0;
        flag =1;
        }
        
        Direct1(0);
        Direct2(1);
        Setpoint1=15;
        Setpoint2=15;
        //Serial.println(direct);
        break;  
      case 5:
      // Forward Right
        Direct1(0);
        Direct2(0);
        Setpoint1=10;
        Setpoint2=20;
        //Serial.println(direct);
        break;  
      case 6:
      // Forward Left
        Direct1(0);
        Direct2(0);
        Setpoint1=20;
        Setpoint2=10;
        //Serial.println(direct);
        break;  
      case 7:
      // Slow Forward
        Direct1(0);
        Direct2(0);
        Setpoint1=10;
        Setpoint2=10;
        //Serial.println(direct);
        break;  
    }
    
  //}   


   
}



