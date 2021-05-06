// This slave file is resposible for taking input and giving output

#include <LiquidCrystal.h>  // for lcd

const int rs = 26, en = 27, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


int BPM=15,TV=300,IE= 2;


void setup()
{
    lcd.begin(16, 2);
    lcd.print("Ventilator");
    delay(2000);
    printData();
}

void loop()
{

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