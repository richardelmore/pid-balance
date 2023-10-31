
#include <LiquidCrystal_I2C.h>
#include <PID_v2.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

// I/O Pins we use
#define POT_PIN    A0
#define SERVO_PIN  9

//Define other constants
#define PID_CYCLE_TIME 100
#define DISTANCE_BUFFER_SIZE 16

// Define globals
const double kP = 0.0;
const double kI = 0.0;
const double kD = 0.0;
int target;

unsigned long nextCycle = 0;                        // time (in millis) when we will next evaluate the PID controller.
int ledValue = 0;                                   // Current value of the LED (0 or 1)
unsigned int distanceBuffer[DISTANCE_BUFFER_SIZE];  // Buffer to store distance values so we can compute a rolling average
unsigned int bufferIndex = 0;                       // Index of next element of buffer to receive new value;
   
Servo servo;
LiquidCrystal_I2C lcd(0x27, 16, 2);                 // Use I2C address 0x27 for 16 character x 2 line display.
PID_v2 pid (kP, kI, kD, PID::Direct);
VL53L0X tof;

void setup()
{
  lcd.init();           // Initialize the display
  lcd.backlight();      // Turn on the backlight.
  lcd.setCursor(0, 0);
  lcd.print("Dist:  "); 
  lcd.setCursor(0, 1);
  lcd.print("Target: "); 

  // Setup serial port for 115200 baud and print initialize message
  Serial.begin(115200);
  Serial.println("Initializing");

  // Configure the LED ping and set now as the last toggle time.
  pinMode(LED_BUILTIN, OUTPUT);
  nextCycle = millis();

  servo.attach(SERVO_PIN);

  Wire.begin();
  tof.setTimeout(500);
  if (!tof.init())
  {
    Serial.println( "ERROR: Could not detect or initialize VL53L0X sensor");
    while(1) {}
  }
  tof.startContinuous();
 
  // Read initial distance and start the PID controller.
  unsigned int currentDistance = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred())
  {
    Serial.println("VL53L0X TIMEOUT");
    while(1) {};
  }

  // Fill the distance buffer with the initial range value
  for (int n = 0; n < DISTANCE_BUFFER_SIZE; n++)
    distanceBuffer[n] = currentDistance;

  // Initialize and start our PID controller.
    target = analogRead(POT_PIN);
    target = map(target, 0, 1023, 100, 400);
  
  pid.SetOutputLimits(0.0, 180.0);
  pid.Start(currentDistance, 90.0, target);
}


void loop()
{
  unsigned long now = millis();

  int newTarget = analogRead(POT_PIN);
  newTarget = map(newTarget, 0, 1023, 100, 400);
  if ( abs(target - newTarget) >= 3 )
  {
    target = newTarget;
    pid.Setpoint(target);
  }
  

  // Since the output of the distance sensor drifts around a bit we use a rolling average to smooth the distance out.
  // Read the distance to the ball
  int currentDistance = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred())
  {
    Serial.println("VL53L0X TIMEOUT");
    return;
  }
    
  if ( currentDistance > 500 )
  {
    Serial.print("Distance out of range: ");
    Serial.println(currentDistance);
  }
  else
  {
    distanceBuffer[bufferIndex++] = currentDistance;
    bufferIndex %= DISTANCE_BUFFER_SIZE;
  }


  // If we have not reached the time to perform our next cycle then return without doing anything else.
  if (now < nextCycle)
  {
    return;
  }

  // Calculate time of next PID cycle
  nextCycle = now + PID_CYCLE_TIME;

  // Toggle the state of the built-in LED
  ledValue = 1 - ledValue;   
  digitalWrite(LED_BUILTIN, ledValue);

  // Calculate current distance average
  unsigned int distance = 0;
  for (int n = 0; n < DISTANCE_BUFFER_SIZE; n++)
  {
    distance += distanceBuffer[n];
  }
  distance /= DISTANCE_BUFFER_SIZE;

  double servoAngle = pid.Run(distance);
  servo.write(servoAngle);
  
  lcd.setCursor(8, 0);
  lcd.print(distance);
  lcd.print("  ");
   
  lcd.setCursor(8, 1);
  lcd.print(target);
  lcd.print("  ");
}
