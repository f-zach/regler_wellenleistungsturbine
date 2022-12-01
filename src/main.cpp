#include <Arduino.h>
#include <Servo.h>
#include <movingAvg.h>

void learnLeverSettings();

Servo ecuOut;

movingAvg throttleAvg(50);
movingAvg brakeAvg(50);

char bufferArr[20] = {'1', '2'};
String message = "";

long tStart = 0, tBlink = 0;
int potBrake = 0, potThrottle = 0, engineRPM = 0, brakeRPM = 0;
float power = 0.0;
int setRPM = 0;
float brakeSetting = 0.0;
float throttleSetting = 0.0;
int mode = 0;
char c, incomingChar;
int pwmValve = 0;

bool controllerOnSwitch, controllerOnSwitchLast, valueRecieved, valueConfirmed, valueDeclined;

int valvePin = 21, ecuPin = 37, turbineOnPin = 29;
int throttleLeverPin = A12, brakeLeverPin = A13;
int maxThrottleValue = 0, minThrottleValue = 9999, maxBrakeValue = 0, minBrakeValue = 9999;

String dataTransmission = "";

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial5.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(ecuPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(27, INPUT);
  pinMode(29, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);

  throttleAvg.begin();
  brakeAvg.begin();

  ecuOut.attach(ecuPin);
  ecuOut.writeMicroseconds(0);

  digitalWrite(13, LOW);

  // Set pwm frequency of the brake valve
  analogWriteFrequency(valvePin, 800);

  // Increase read and write resolution ofh the analog and pwm pins
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

  
  if (Serial1.available() > 10)
  {
    delay(2);
    Serial1.readStringUntil('*');
    do
    {
      incomingChar = Serial1.read();
      //Serial.println(incomingChar);
      switch (incomingChar)
      {
      case 'r':
        engineRPM = Serial1.readStringUntil(';').toFloat();
        //Serial.println(engineRPM);
        break;

      case 's':
        brakeRPM = Serial1.readStringUntil(';').toFloat();
        //Serial.println(brakeRPM);
        break;

      case 'p':
        power = Serial1.readStringUntil(';').toFloat();
        //Serial.println(power);
        break;

      default:
        break;
      }
      delayMicroseconds(10);
    } while (incomingChar != '#');

  }

  

  // Go to mode 1: Controller switch toggled on (wait for desired RPM input)
  if (controllerOnSwitch && !controllerOnSwitchLast)
  {
    digitalWrite(26, HIGH);
    mode = 1;
  }
  // Go to moode 2: Controller switch on and waiting for user confirmation of RPM value
  else if (controllerOnSwitch && controllerOnSwitchLast && valueRecieved)
  {
    mode = 2;
    valueRecieved = false;
  }
  // Go to mode 3: Controller switch on and value confirmed by the user -> controller is activated
  else if (controllerOnSwitch && controllerOnSwitchLast && valueConfirmed)
  {
    mode = 3;
    valueConfirmed = false;
  }
  // Go to mode 0: User declines RPM value
  else if (controllerOnSwitch && controllerOnSwitchLast && valueDeclined)
  {
    mode = 0;
    valueDeclined = false;
  }
  // Go to mode 0: Controller switch toggled off
  else if (!controllerOnSwitch && controllerOnSwitchLast)
  {
    digitalWrite(26, LOW);
    mode = 0;
  }


  controllerOnSwitchLast = controllerOnSwitch;

// Read analog values of throttle and brake lever
  potBrake = brakeAvg.reading(analogRead(A13));
  potThrottle = throttleAvg.reading(analogRead(A12));

// Check if throttle lever is in the deadzone
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
    // Calculate the throttle setting
    throttleSetting = (float(potThrottle) - minThrottleValue - 30) / (maxThrottleValue - minThrottleValue - 60);
  }

// Check if brake lever is in the deadzone
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
    //Calculate the brake setting
    brakeSetting = (float(potBrake) - minBrakeValue - 30) / (maxBrakeValue - minBrakeValue - 60);
  }
  // If turbine switch is on write the servo signal to be sent to the ecu
  if(digitalRead(29))
  {
    ecuOut.writeMicroseconds(995 + 115 + int(throttleSetting * 885));
  }
  else if(!digitalRead(29))
  {
    // If the turbine switch is on, always send this value
    ecuOut.writeMicroseconds(995);
  }

  // Apply the brake valve setting to the PWM signal
  analogWrite(valvePin, int(brakeSetting * 4096));

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

  //Send the data to be displayed to the console
  if (millis() - tStart >= 100)
  {
    dataTransmission = "";
    dataTransmission = "*";
    dataTransmission += +"b" + String(brakeSetting,4) + ";";
    dataTransmission += +"t" + String(throttleSetting,4) + ";";
    dataTransmission += +"o" + String(int(brakeSetting * 4096)) + ";";
    dataTransmission += +"p" + String(power) + ";";
    dataTransmission += +"r" + String(brakeRPM) + ";";
    dataTransmission += +"s" + String(setRPM) + ";";
    dataTransmission += +"m" + String(mode) + ";";
    dataTransmission += +"#";
    Serial.println(dataTransmission);
    Serial5.println(dataTransmission);
    tStart = millis();
  }

  //Serial.println(digitalRead(29));
}


// On startup this function is used to calibrate the throttle and brake lever
void learnLeverSettings()
{
  int reading = 0;
  // Learn max throttle setting
  Serial.println("Set throttle to full!");

  while (analogRead(throttleLeverPin) < 3200)
  {
    Serial5.println("*m4;n0;#");
  }


  Serial.println("Wait!");

  int tStartReading = millis();
  int lastReading = 0;
  do
  {
    reading = analogRead(throttleLeverPin);
    if (reading > lastReading)
    {
      maxThrottleValue = reading;
    }
    Serial5.println("*m5;n0;#");
  } while (millis() - tStartReading < 3000);

  Serial.println(maxThrottleValue);
  Serial5.println("*m6;n0;#");
  delay(1200);

  // Read min throttle Setting
  Serial.println("Set throttle to zero!");

  while (analogRead(throttleLeverPin) > 2500)
  {
    Serial5.println("*m4;n1;#");
  }

  Serial.println("Wait!");

  tStartReading = millis();
  lastReading = 9999;
  do
  {
    int reading = analogRead(throttleLeverPin);
    if (reading < lastReading)
    {
      minThrottleValue = reading;
    }
    Serial5.println("*m5;n1;#");
  } while (millis() - tStartReading < 3000);

  Serial.println(minThrottleValue);
  Serial5.println("*m6;n1;#");
  delay(1200);

  // Set max brake setting

  Serial.println("Set brake to full!");

  while (analogRead(brakeLeverPin) < 2800)
  {
    Serial5.println("*m4;n2;#");
  }

  Serial.println("Wait!");

  tStartReading = millis();
  lastReading = 0;
  do
  {
    int reading = analogRead(brakeLeverPin);
    if (reading > lastReading)
    {
      maxBrakeValue = reading;
    }
    Serial5.println("*m5;n2;#");
  } while (millis() - tStartReading < 3000);

  Serial.println(maxBrakeValue);
  Serial5.println("*m6;n2;#");
  delay(1200);

  // Set min brake setting
  Serial.println("Set brake to zero!");

  while (analogRead(brakeLeverPin) > 2000)
  {
    Serial5.println("*m4;n3;#");
  }

  Serial.println("Wait!");

  tStartReading = millis();
  lastReading = 9999;
  do
  {
    int reading = analogRead(brakeLeverPin);
    if (reading < lastReading)
    {
      minBrakeValue = reading;
    }
    Serial5.println("*m5;n3;#");
  } while (millis() - tStartReading < 3000);

  Serial.println(minBrakeValue);
  Serial5.println("*m6;n3;#");

  delay(1200);
}