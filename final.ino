#include <NewPing.h>

unsigned int sensor[10];/*sensor readings are saved here*/
unsigned int sensorWhileLEDoff[10]; /* sensor reading while LED is off*/
boolean firstData[10];/*to eliminate effect of noise*/
byte sensorPin[10] = {40,38,36,34,32,30,28,26,24,22};/*arduino pins to read sensors*/
byte NumOfSensor = 10;
byte i; /*just to run for loop!!*/
unsigned int MaxWaitTime = 4000; /*equivalent to 12 bit ADC*/
int IR_LED_ON_PIN = 23;

#define MotorBp 11
#define MotorBn 12
#define MotorB 13

#define MotorAp 10
#define MotorAn 9
#define MotorA 8

int dis;
#define MAX_DISTANCE 200
#define TRIGGER_PIN  5  
#define ECHO_PIN     6
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


int th[10]={100,100,100,100,100,100,100,100,100,100};

int sen[10],s[10], lastSensor,lastError;

int base_L=100;
int base_R=100;
int kp=15;
int kd=8;
int error,sum,p,d,corr;

  
void setup()
{
  Serial.begin(9600);
  pinMode(IR_LED_ON_PIN,OUTPUT);
  
  mot_init();
  delay(1000);
}

void loop()
{
  dis = sonar.ping_cm();
  if (dis <= 10 && dis != 0) 
  {
    delay(30);
    rightturn();
    delay(150);
    wheel_drive(100,100);
    delay(500);
    leftturn();
    delay(150);
    wheel_drive(100,100);
    delay(500);
    leftturn();
    delay(150);
    wheel_drive(100,100);
    delay(500);
    rightturn();
    delay(150);
  }

  
  line_follow();
}


void rightturn()
{
  wheel_drive(100,-100);
  delay(350);
}

void leftturn()
{
  wheel_drive(-100,100);
  delay(350);
}

void mot_init()
{
  pinMode(MotorBp,OUTPUT);
  pinMode(MotorBn,OUTPUT);
  pinMode(MotorAp,OUTPUT);
  pinMode(MotorAn,OUTPUT);
  pinMode(MotorB,OUTPUT);
  pinMode(MotorA,OUTPUT);
}

void line_follow()
{


delay(75);
digitalWrite(MotorBp,HIGH);
digitalWrite(MotorBn,HIGH);
digitalWrite(MotorAp,HIGH);
digitalWrite(MotorAn,HIGH);
delay(75);


digitalWrite(IR_LED_ON_PIN, LOW); // turn ON IR LEDs
delay(20); // give some time to turn ON
readSensor(); // read line sensors while IR LEDs are OFF
// store current sensor data to sensorWhileLEDoff
for (i = 0; i <= 9; i++) sensorWhileLEDoff[i] = MaxWaitTime - sensor[i];
digitalWrite(IR_LED_ON_PIN, HIGH); // turn ON IR LEDs
delay(20); // give some time to turn ON
readSensor(); // read line sensors while IR LEDs are ON
// calculate external light compensated readings
for (i = 0; i <= 9; i++) sensor[i] = sensor[i] + sensorWhileLEDoff[i];
showsensorData();
  
//  readSensor();
  if(error==420)
  {
    if(lastSensor==1) wheel_drive(-100,100);
    else if(lastSensor==2) wheel_drive(100,-100);
  }
  else 
  { 
    p=kp*error;
    d=kd*(error-lastError);
    corr=p+d;
//   Serial.println(corr);
    wheel_drive(base_L+corr,base_R-corr);
    if((error-lastError)!=0) delay(5);
    lastError=error;
  }
}

void readSensor()
{

  
for (i = 0; i < NumOfSensor; i++)
{
digitalWrite(sensorPin[i], HIGH);
pinMode(sensorPin[i], OUTPUT);
}
delayMicroseconds(10);
for (i = 0; i < NumOfSensor; i++)
{
pinMode(sensorPin[i], INPUT);
digitalWrite(sensorPin[i], LOW);
sensor[i] = MaxWaitTime;
firstData[i] = false;
}
unsigned long startTime = micros();
while (micros() - startTime < MaxWaitTime)
{
unsigned int time = micros() - startTime;
for (i = 0; i < NumOfSensor; i++)
{
if ((digitalRead(sensorPin[i]) == LOW) && (firstData[i] == false))
{
sensor[i] = time;
firstData[i] = true;
}

if(sensor[i]<th[i])
    {
      s[i]=0;
    }
    else { 
      s[i]=1;
    }

}
}

  sum=s[9]+s[7]+s[5]+s[3]+s[1];
  if(sum!=0)
    error=(s[9]*10 + s[7]*20 + s[5]*30 + s[3]*40 + s[1]*50) /sum -30;
    
  else
     error=420;
  
  if(s[1]==1) lastSensor=1;
  else if(s[9]==1) lastSensor=2;
//  Serial.println(error);
}


void showsensorData()
{
for (i = 0; i < NumOfSensor; i++)
{
Serial.print(sensor[i]);
Serial.print(" ");
}
Serial.println();
}

 
void wheel_drive(int lms, int rms)
{
  if (lms > 255) lms = 255;
  else if (lms < -255) lms = -255;
  if (rms > 255) rms = 255;
  else if (rms < -255) rms = -255;
  
  if (lms == 0)
  {
    digitalWrite(MotorBp,HIGH);
    digitalWrite(MotorBn,HIGH);
  }
  else if (lms > 0)
  {
    digitalWrite(MotorBp,HIGH);
    digitalWrite(MotorBn,LOW);
  }
  else if (lms < 0)
  {
    digitalWrite(MotorBp,LOW);
    digitalWrite(MotorBn,HIGH);
  }

  if (rms == 0)
  {
    digitalWrite(MotorAp,HIGH);
    digitalWrite(MotorAn,HIGH);
  }
  else if (rms > 0)
  {
    digitalWrite(MotorAp,HIGH);
    digitalWrite(MotorAn,LOW);
  }
  else if (rms < 0)
  {
    digitalWrite(MotorAp,LOW);
    digitalWrite(MotorAn,HIGH);
  }
  
  analogWrite(MotorA,abs(rms));
  analogWrite(MotorB,abs(lms));
}
