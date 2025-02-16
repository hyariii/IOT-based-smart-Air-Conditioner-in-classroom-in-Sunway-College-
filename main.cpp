#include <LiquidCrystal_I2C.h>
#include <Servo.h>  // Include Servo library

LiquidCrystal_I2C lcd1(0x20, 16, 2);
LiquidCrystal_I2C lcd2(0x21, 16, 2);

#define LEDPIN 4
#define PIRPIN 5

#define GREEN_LED 8
#define YELLOW_LED 7
#define RED_LED 6

#define COOLING_MOTOR_PIN 9
#define HEATING_MOTOR_PIN 10

const int analogIn = A0; 
const int humiditySensorPin = A1;
const int airQualitySensorPin = A2;

Servo windowServo; 
int servoPin = 11;

int RawValue = 0;
double Voltage = 0;
double tempC = 0;
int airQualityReading = 0;

const int hot = 30;
const int cold = 22;

bool systemActive = false;
unsigned long lastMotionTime = 0;
unsigned long motionTimeout = 60000;

void setup() {
  pinMode(LEDPIN, OUTPUT);
  pinMode(PIRPIN, INPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(COOLING_MOTOR_PIN, OUTPUT);
  pinMode(HEATING_MOTOR_PIN, OUTPUT);

  Serial.begin(9600);

  lcd1.init();
  lcd1.backlight();

  lcd2.init();
  lcd2.backlight();

  // Attach the servo to the pin
  windowServo.attach(servoPin);
  windowServo.write(0); // Close the window (servo at 0°)
}

void loop() {
  if (digitalRead(PIRPIN) == HIGH) {
    lastMotionTime = millis();
    systemActive = true;
  } else if (millis() - lastMotionTime > motionTimeout) {
    systemActive = false;
  }

  if (!systemActive) {
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("No Motion Detected.");
    lcd1.setCursor(0, 1);
    lcd1.print("Turning off AC.");
    delay(2000);

    lcd1.clear();
    lcd2.clear();
    digitalWrite(LEDPIN, LOW);
    digitalWrite(COOLING_MOTOR_PIN, LOW);
    digitalWrite(HEATING_MOTOR_PIN, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    return;
  }

  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1023.0) * 5000;
  tempC = (Voltage - 500) / 10.0;
  tempC = constrain(tempC, -50, 50);

  int humiditySensorOutput = analogRead(humiditySensorPin);
  int humidity = map(humiditySensorOutput, 0, 1023, 10, 70);

  airQualityReading = analogRead(airQualitySensorPin);
  float airQuality = map(airQualityReading, 85, 370, 0, 500);

  lcd1.setCursor(0, 0);
  lcd1.print("Temp: ");
  lcd1.print(tempC, 1);
  lcd1.print("C   ");

  if (tempC > hot) {  
  lcd1.setCursor(0, 1);
  lcd1.print("Hot! Cooling ON   ");
  digitalWrite(COOLING_MOTOR_PIN, HIGH);
  digitalWrite(HEATING_MOTOR_PIN, LOW);
  } 
  else if (tempC < cold) {  
    if (humidity > 60) {  
          lcd1.setCursor(0, 1);
      lcd1.print("Humid! Cooling ON ");
      digitalWrite(COOLING_MOTOR_PIN, HIGH);
      digitalWrite(HEATING_MOTOR_PIN, LOW);
    } else {  
      // Low temperature + Low humidity → Hot air (heating)
      lcd1.setCursor(0, 1);
      lcd1.print("Cold! Heating ON ");
      digitalWrite(COOLING_MOTOR_PIN, LOW);
      digitalWrite(HEATING_MOTOR_PIN, HIGH);
    }
  } 
  else if (humidity > 60) {  
    // Normal temperature (22-30°C) + High humidity → Cooling ON
    lcd1.setCursor(0, 1);
    lcd1.print("High Humidity! Cooling ON ");
    digitalWrite(COOLING_MOTOR_PIN, HIGH);
    digitalWrite(HEATING_MOTOR_PIN, LOW);
  } 
  else {  
    // If temperature is normal and humidity is low, turn everything OFF
    lcd1.setCursor(0, 1);
    lcd1.print("Normal Temp.        ");
    digitalWrite(COOLING_MOTOR_PIN, LOW);
    digitalWrite(HEATING_MOTOR_PIN, LOW);
  }


  // Display humidity and air quality in LCD2
  lcd2.setCursor(0, 0);
  lcd2.print("Humidity: ");
  lcd2.print(humidity);
  lcd2.print("%   ");

  lcd2.setCursor(0, 1);
  lcd2.print("Air Quality: ");
  lcd2.print(airQuality);
  lcd2.print("   ");

  // Control LEDs for air quality
  // Control LEDs for air quality and servo
if (airQuality < 100) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    windowServo.write(0); // Close the window if air quality is good
} else if (airQuality < 200) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    windowServo.write(0); // Keep the window closed if air quality is moderate
} else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    windowServo.write(90); // Open the window if air quality is bad
}

  digitalWrite(LEDPIN, HIGH);

  delay(500);}
