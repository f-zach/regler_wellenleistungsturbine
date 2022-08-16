#include <Arduino.h>
#include <Servo.h>
#include <movingAvg.h>

void learnLeverSettings();

Servo ecuOut;

movingAvg throttleAvg(10);
movingAvg brakeAvg(10);

char bufferArr[20] = {'1', '2'};
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
int throttleLeverPin = A12, brakeLeverPin = A13;
int maxThrottleValue = 0, minThrottleValue = 9999, maxBrakeValue = 0, minBrakeValue = 9999;

String dataTransmission = "";

void setup()
{
  Serial.begin(9600);
  Serial5.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(ecuPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(27, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);

  throttleAvg.begin();
  brakeAvg.begin();

  ecuOut.attach(ecuPin);

  digitalWrite(13, LOW);

  analogWriteFrequency(valvePin, 800);
  analogWriteResolution(12);
  analogReadResolution(12);

  learnLeverSettings();
}

void loop()
{

  controllerOnSwitch = digitalRead(27);

  if (Serial5.available() > 0)
  {
    c = Serial5.read();
    switch (c)
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

  //Go to mode 1: Controller switch toggled on (wait for desired RPM input)
  if (controllerOnSwitch && !controllerOnSwitchLast)
  {
    digitalWrite(26, HIGH);
    mode = 1;
  }
  //Go to moode 2: Controller switch on and waiting for user confirmation of RPM value
  else if (controllerOnSwitch && controllerOnSwitchLast && valueRecieved)
  {
    mode = 2;
    valueRecieved = false;
  }
  //Go to mode 3: Controller switch on and value confirmed by the user -> controller is activated
  else if (controllerOnSwitch && controllerOnSwitchLast && valueConfirmed)
  {
    mode = 3;
    valueConfirmed = false;
  }
  //Go to mode 0: User declines RPM value
  else if (controllerOnSwitch && controllerOnSwitchLast && valueDeclined)
  {
    mode = 0;
    valueDeclined = false;
  }
  //Go to mode 0: Controller switch toggled off
  else if (!controllerOnSwitch && controllerOnSwitchLast)
  {
    digitalWrite(26, LOW);
    mode = 0;
  }

  controllerOnSwitchLast = controllerOnSwitch;

  potBrake = brakeAvg.reading(analogRead(A13));
  potThrottle = throttleAvg.reading(analogRead(A12));

  if (potThrottle < (minThrottleValue + 30))
  {
    throttleSetting = 0.0;
  }
  else if (potThrottle > (maxThrottleValue - 30))
  {
    throttleSetting = 1.0;
  }
  else
  {
    throttleSetting = (float(potThrottle) - minThrottleValue - 30) / (maxThrottleValue - minThrottleValue - 60);
  }

  if (potBrake < (minBrakeValue + 30))
  {
    brakeSetting = 0.0;
  }
  else if (potBrake > (maxBrakeValue - 30))
  {
    brakeSetting = 1.0;
  }
  else
  {
    brakeSetting = (float(potBrake) - minBrakeValue - 30) / (maxBrakeValue - minBrakeValue - 60);
  }

  ecuOut.writeMicroseconds(995 + int(throttleSetting * 1000));
  analogWrite(valvePin, int(brakeSetting * 4096));

  rpm = int(throttleSetting * 150000);
  power = brakeSetting * 13;

  switch (mode)
  {
  case 0:
    digitalWrite(13, LOW);
    break;

  case 1:
    if (millis() - tBlink > 1000)
    {
      digitalToggle(13);
      tBlink = millis();
    }
    break;

  case 2:
    if (millis() - tBlink > 500)
    {
      digitalToggle(13);
      tBlink = millis();
    }
    break;

  case 3:
    digitalWrite(13, HIGH);
    break;

  default:
    digitalWrite(13, LOW);
    break;
  }

  if (millis() - tStart >= 100)
  {
    dataTransmission = "";
    dataTransmission = "*";
    dataTransmission += +"b" + String(brakeSetting) + ";";
    dataTransmission += +"t" + String(throttleSetting) + ";";
    dataTransmission += +"p" + String(power) + ";";
    dataTransmission += +"r" + String(rpm) + ";";
    dataTransmission += +"s" + String(setRPM) + ";";
    dataTransmission += +"m" + String(mode) + ";";
    dataTransmission += +"#";
    //Serial.println(dataTransmission);
    Serial5.println(dataTransmission);
    tStart = millis();
  }
}

void learnLeverSettings()
{
  int reading = 0;
  //Learn max throttle setting
  if (analogRead(throttleLeverPin) < 3200)
  {
    Serial.println("Set throttle to full!");
  }

  while (analogRead(throttleLeverPin) < 3200)
  {
    Serial5.println("!"+String(analogRead(throttleLeverPin)))
  }

  int tStartReading = millis();
  int lastReading = 0;
  do
  {
    reading = analogRead(throttleLeverPin);
    if (reading > lastReading)
    {
      maxThrottleValue = reading;
    }
  } while (millis() - tStartReading < 3000);

  Serial.println(maxThrottleValue);

  //Read min throttle Setting
  if (analogRead(throttleLeverPin) > 2500)
  {
    Serial.println("Set throttle to zero!");
  }

  while (analogRead(throttleLeverPin) > 2500)
  {
  }

  tStartReading = millis();
  lastReading = 9999;
  do
  {
    int reading = analogRead(throttleLeverPin);
    if (reading < lastReading)
    {
      minThrottleValue = reading;
    }
  } while (millis() - tStartReading < 3000);

  Serial.println(minThrottleValue);

  //Set max brake setting
  if (analogRead(brakeLeverPin) < 2800)
  {
    Serial.println("Set brake to full!");
  }

  while (analogRead(brakeLeverPin) < 2800)
  {
  }

  tStartReading = millis();
  lastReading = 0;
  do
  {
    int reading = analogRead(brakeLeverPin);
    if (reading > lastReading)
    {
      maxBrakeValue = reading;
    }
  } while (millis() - tStartReading < 3000);

  Serial.println(maxBrakeValue);

  //Set min brake setting
  if (analogRead(brakeLeverPin) > 2000)
  {
    Serial.println("Set brake to zero!");
  }

  while (analogRead(brakeLeverPin) > 2000)
  {
  }

  tStartReading = millis();
  lastReading = 9999;
  do
  {
    int reading = analogRead(brakeLeverPin);
    if (reading < lastReading)
    {
      minBrakeValue = reading;
    }
  } while (millis() - tStartReading < 3000);

  Serial.println(minBrakeValue);
}