#include <Arduino.h>

char bufferArr[20] = {'1','2'};
String message = "";

long tStart = 0, tBlink = 0;
int potBrake = 0;
int potThrottle = 0;
float power = 0.0;
int rpm = 0;
int setRPM = 100000;
float brakeSetting = 0.0;
float throttleSetting = 0.0;
int mode = 0;
char c;
int pwmValve = 0;

bool controllerOnSwitch, controllerOnSwitchLast, valueRecieved, valueConfirmed, valueDeclined;

int valvePin = 21, ecuPin = 37;

String dataTransmission = "";

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  pinMode(13,OUTPUT);
  pinMode(26,OUTPUT);
  pinMode(ecuPin,OUTPUT);
  pinMode(valvePin,OUTPUT);
  pinMode(27,INPUT);
  pinMode(31,INPUT);
  pinMode(32,INPUT);

  digitalWrite(13,LOW);

  analogWriteFrequency(valvePin,800);

  analogWrite(ecuPin,127);
  analogWrite(valvePin, 0);

}

void loop() {
  
  controllerOnSwitch = digitalRead(27);

  if(Serial5.available() > 0)
  {
    c = Serial5.read();
    switch(c)
    {
      case '*':
        Serial.println('*');
        setRPM = Serial5.readStringUntil(';').toFloat();        
        valueRecieved = true;
        valueConfirmed = false;
        break;
      case '#':
        Serial.println('#');
        valueConfirmed = true;
        break;

      case '+':
        Serial.println('+');
        mode = 1;
        valueConfirmed = false;
        break;

     case '-':
        Serial.println('-');
        valueDeclined = true;
        break;
    }
  }  

  if(controllerOnSwitch && !controllerOnSwitchLast)
  {
    digitalWrite(26,HIGH);
    mode = 1;
  }
  else if(controllerOnSwitch && controllerOnSwitchLast && valueRecieved)
  {
    mode = 2;
    valueRecieved = false;
  }
  else if(controllerOnSwitch && controllerOnSwitchLast && valueConfirmed)
  {
    mode = 3;
    valueConfirmed = false;
  }
  else if(controllerOnSwitch && controllerOnSwitchLast && valueDeclined)
  {
    mode = 0;
    valueDeclined = false;
  }
  else if(!controllerOnSwitch && controllerOnSwitchLast)
  {
    digitalWrite(26,LOW);
    mode = 0;
  }

  controllerOnSwitchLast = controllerOnSwitch;
  
  potBrake = analogRead(A12);  
  potThrottle = analogRead(A13);

  Serial.println(potBrake);

  pwmValve = potBrake/4;
  Serial.println(pwmValve);
  

  if(pwmValve < 10)
  {
    analogWrite(valvePin,0);
  }
  else
  {
    analogWrite(valvePin,pwmValve);
  }

  throttleSetting = float(potThrottle)/1023;
  brakeSetting = float(potBrake)/1023;

  rpm = int(throttleSetting * 150000);
  power = brakeSetting * 13;  

  switch(mode)
  {
    case 0:
      digitalWrite(13,LOW);
      break;
      
    case 1:
      if(millis()-tBlink > 1000)
      {
        digitalToggle(13);
        tBlink = millis();
      }
      break;
      
    case 2:
      if(millis()-tBlink > 500)
      {
        digitalToggle(13);
        tBlink = millis();
      }
      break;
      
    case 3:
      digitalWrite(13,HIGH);
      break;

    default:
      digitalWrite(13, LOW);
      break;    
  }

  if(millis() - tStart >= 100)
  {
     dataTransmission = "";
     dataTransmission = "*";
     dataTransmission += + "b" + String(brakeSetting) + ";";
     dataTransmission += + "t" + String(throttleSetting) + ";";
     dataTransmission += + "p" + String(power) + ";";
     dataTransmission += + "r" + String(rpm) + ";";
     dataTransmission += + "s" + String(setRPM) + ";";
     dataTransmission += + "m" + String(mode) + ";";  
     dataTransmission += + "#";
     Serial.println(dataTransmission);
     Serial5.println(dataTransmission);
     tStart = millis();
  }
  

  
}