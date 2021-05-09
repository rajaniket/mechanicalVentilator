// This master file is responsible for running the motor according to given parameter

// for I2C communication
# include<Wire.h>

#define dirPin 8  // direction pin of steeper motor
#define stepPin 9 // step pin of steeper motor
#define limit 3  //  limit switch
#define button 2  // for play/pause button
unsigned long currTime;

int Speed = 50;
int table[] = {0,0,0,0,0,0,0,0};  // init table for storing collecting controling parameter
int tv=500;   // Tidle volume
int bpm=20;   // Breath per minute
int ie=2;   // I to E ratio 
int control = 0;
int n = 4669;  // number of step for steeper motor
float T;    // Time period for inhelation and exhalation
float Tin;  // inhelation time
float Tex;  // Exhalation time
float Dti = 105.84;  // delay in each stape(in microsecond)
float Dte = 423.36; // delay in each stape(in microsecond)

bool flag = false;
bool play = false;

void setup()
{
  pinMode(limit,INPUT_PULLUP);  // for limit switch
  pinMode(2,INPUT); //for paly/pause
  attachInterrupt(0,buttonPressed,FALLING); // attach interrupt for play/pause
  pinMode(stepPin, OUTPUT);  // motor step pins
  pinMode(dirPin, OUTPUT);   // motor direction pins
  Serial.begin(9600);
  delay(1000);
  Wire.begin(); // join i2c bus (address optional for master)
  
  // Homing
  if(digitalRead(limit)==1)
  {
    digitalWrite(dirPin, HIGH);
    while(digitalRead(limit)==1)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(Speed);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(Speed);
    }
  }
  delay(3000);
  currTime = millis();
}

void loop()
{
  int one =1;
  if(play){
    if(flag){
      Wire.requestFrom(0x08,8);
      for(int i=0;i<8;i++){
        int c = Wire.read();
        table[i]=c; 
      }
      int temp1 = (table[2]<<8)|table[3];
      int temp2 = (table[4]<<8)|table[5];
      int temp3 = (table[6]<<8)|table[7];
      if(temp1<50 & temp2<10){
        tv = (table[0]<<8)|table[1];
        if(tv>900|tv<100){
          tv=100;
        }
        //Serial.println(temp3);
        bpm = temp1;
        ie = temp2;
        control = temp3;
        n = 1751.6 + (4.8974*tv);
        T = 60000000/bpm; //in microsecond
        Tin = T/(1+ie);  //in microsecond
        Tex = (T*ie)/(1+ie);
        Dti = (Tin)/n; 
        Dte = (Tex)/n;
      }
    }
    flag =true;
    float timeOut = (millis()-currTime)*1000; // converting timeOut in to microsecond
    Serial.println(control);
    if(timeOut>T | control==1){ //timeOut>T |
      control=0;
      one=0;
      currTime = millis();
      // For Inspiration
      digitalWrite(dirPin, LOW);
      for (int i = 0; i < n; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(Dti/2); 
        digitalWrite(stepPin, LOW);
        delayMicroseconds(Dti/2);
      }
      // For Expiration
      digitalWrite(dirPin, HIGH);
      for (int i = 0; i < n; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(Speed);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(Speed);
      }
      Serial.println("Motor is running....");
    }
  }
  else{
    Serial.println("Motor is paused....");
    delay(2000);
  }

}

void buttonPressed()
{
  if(play){
    play = false;  // if motor is ON then OFF
    digitalWrite(13,LOW);
  }else{
    play = true;  // if motor is OFF then ON the motor
    digitalWrite(13,HIGH);
  }
}