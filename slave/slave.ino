// This slave file is resposible for taking input and giving output

#include <EEPROM.h> // for EEPROM memory
#include <Wire.h>   //for I2c Communication
#include <LiquidCrystal.h>  // for lcd

// SPI communiation for pressure sensor
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCL 13
#define BMP_SDO 12
#define BMP_SDA 11
#define BMP_CSB1 10
#define BMP_CSB2 9
Adafruit_BMP280 bmp2(BMP_CSB1, BMP_SDA, BMP_SDO, BMP_SCL); // bmp2 -> near mouth
Adafruit_BMP280 bmp1(BMP_CSB2, BMP_SDA, BMP_SDO, BMP_SCL); // bmp1 -> near ambubag
float alpha = 0.05;   // exponential smoothing constant
float smooth_pa1; // initialization at generic value
float pa1;  //for reading pressure sensor data
float smooth_pa2; // initialization at generic value
float pa2;  //for reading pressure sensor data
float m = 20;
float flow;
float smooth_flow;


const int rs = 26, en = 27, d4 = 25, d5 = 24, d6 = 23, d7 = 22;  // lcd pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// variable for sending data to master device
uint8_t table[4];

int control = 0;
#define alarm 42  // for alarm
// Defining pin for encoder
int CLK3 = 30 , CLK2=5 , CLK1=7;
int DT3 = 4 , DT2=6 , DT1=8;

// input parameter 
int BPM=15, PEEP=5,MODE=0,TV=300;
int IE = 2;
int CLKlast1 , CLKlast2 , CLKlast3;
int aVal1 , aVal2 , aVal3;
int dt;
// int BPM=15,TV=300,IE= 2;

// for BT module
#include <SoftwareSerial.h>
SoftwareSerial mySerial(A8, A9); 
char  Data; 
String text="";
bool temp=false;


void setup()
{
    lcd.begin(16, 2);
    lcd.print("Sanjivini");
    delay(2000);
    printData();
    Wire.begin(0x08);//setup slave device address
    Wire.onRequest(sendData); //Call senData function when get request from master
    pinMode(CLK1,INPUT);
    pinMode(CLK2,INPUT);
    pinMode(CLK3,INPUT);
    pinMode(DT1,INPUT);
    pinMode(DT2,INPUT);
    pinMode(DT3,INPUT);
    pinMode(44,OUTPUT); // for play/pause
    pinMode(alarm,OUTPUT); // for alarm
    CLKlast1 = digitalRead(CLK1);
    CLKlast2 = digitalRead(CLK2);
    CLKlast3 = digitalRead(CLK3);
    attachInterrupt(5,playPause,FALLING); // attach interrupt for play/pause
    attachInterrupt(4,Mode1,FALLING); // for mode 1
    attachInterrupt(1,Mode2,FALLING); // for mode 2
    Serial.begin(9600);
    mySerial.begin(9600);
    // Creating beep sound when is ready to use
    digitalWrite(alarm,HIGH);
    delay(1000);
    digitalWrite(alarm,LOW);
}

void loop()
{
    // **** For I to E Ratio *******
    aVal1 = digitalRead(CLK1);
    if(aVal1 != CLKlast1)
    {
        dt = digitalRead(DT1);
        if( dt != aVal1)
        {
          // AntiCockWise Conter Condition
          if(dt==1 && aVal1==0){
            if(IE <3){
              IE=IE+1;
            }
            printData();
          }
        }
        else
        {
          // ClockWise Counter Condition
            if(dt==0 && aVal1==0){
              if(IE>1){
                IE=IE-1;
              }
              printData();
            }
        }
    }
    CLKlast1 = aVal1;


 //********** For Tidle Volume **********

    aVal2 = digitalRead(CLK2);
    if(aVal2 != CLKlast2)
    {
        dt = digitalRead(DT2);
        if( dt != aVal2)
        {
          // AntiCockWise Conter Condition
          if(dt==1 && aVal2==0){
            if(TV <800)
            {
              TV=TV+50;
            }
              printData();
        }
        }
        else
        {
          // ClockWise Counter Condition
            if(dt==0 && aVal2==0){
            if(TV>100)
            {
              TV=TV-50;
            }
              printData();
        }
        }
    }
    CLKlast2 = aVal2;

 //********** For Breath Per Minute(BPM) **********

    aVal3 = digitalRead(CLK3);
    if(aVal3 != CLKlast3)
    {
        dt = digitalRead(DT3);
        if( dt != aVal3)
        {
          // AntiCockWise Conter Condition
          if(dt==1 && aVal3==0){
            if(BPM <35)
            {
              BPM=BPM+1;
            }
              printData();
            
        }
        }
        else
        {
          // ClockWise Counter Condition
            if(dt==0 && aVal3==0){
                if(BPM>10)
                {
                    BPM=BPM-1;
                }
                printData();
            }
        }
    }
    CLKlast3 = aVal3;


 //******* BT MODULE ******

 int countvar = 1;
  while(mySerial.available()){
    delay(10);
    Data=mySerial.read();
    if(Data != ','){
    text+=Data;
    }
    else{
      if(countvar==1){
        TV = text.toInt();
      }
      else if(countvar==2){
        IE = text.toFloat();
      }
      else if(countvar==3){
        BPM = text.toInt();
      }
      else if(countvar==4){
        PEEP = text.toInt();
      }
      else if(countvar==5){
        MODE = text.toInt();
      }
      countvar++;
      text = "";
    }
    temp=true;
  }
  if(temp){
      printData();
    text = "";
    temp=false;
  }
    

}


void printData(){ // this function for printing data on lcd
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("I:E");
    lcd.setCursor(0,1);
    lcd.print("1:");
    lcd.setCursor(2,1);
    lcd.print(IE);
    lcd.setCursor(6,0);
    lcd.print("TV");
    lcd.setCursor(6,1);
    lcd.print(TV);
    lcd.setCursor(12,0);
    lcd.print("BPM");
    lcd.setCursor(12,1);
    lcd.print(BPM);
}

void sendData(){
    //called on request of master
}

void playPause(){
    // for stop/run the motor
}

void Mode1(){
    // called when mode-1 button is pressed
}

void Mode2(){
    // called when mode-2 button is pressed
}