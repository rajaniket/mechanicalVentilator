// This master file is responsible for running the motor according to given parameter


#define dirPin 8  // direction pin of steeper motor
#define stepPin 9 // step pin of steeper motor



int n = 4669;  // number of step for steeper motor
float Dti = 105.84;  // delay in each stape(in microsecond)
float Dte = 423.36; // delay in each stape(in microsecond)

void setup()
{
    pinMode(stepPin, OUTPUT);  // motor step pins
    pinMode(dirPin, OUTPUT);   // motor direction pins
    Serial.begin(9600);
    delay(1000);
}

void loop()
{
    // for Inspiration
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
      delayMicroseconds(Dte/2);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(Dte/2);
    }
    Serial.print("Motor is running......");
}