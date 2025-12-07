#include <Wire.h>   // Added so we can Establish I2C Communication.
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h> // Include the Motor Servo Driver Library

// SoftwareSerial BT defines a variable to utilize serial communication protocol methods via Transmission (TX) and Receiving (RX) pins between the bluetooth module and the arduino uno. 
SoftwareSerial BT(10, 11); // Pin 10 = RX of Arduino (to TX of Bluetooth module), 11 = TX of Arduino (to RX of Bluetooth module via voltage divider)
Adafruit_PWMServoDriver Servos_Drive = Adafruit_PWMServoDriver();

// We define the minimum and maximum pulse length counts.
#define SERVO_MIN 102 // Corresponds to 0 degrees. (!!For Testing)
#define SERVO_MAX 512 // Corresponds to 180 degrees. (!!For testing)

// Initializing the angles for all the servo motors.
volatile long base_angle = 95;
volatile long motor2_angle = 90;
volatile long motor3_angle = 100;
volatile long motor4_angle = 90;
volatile long motor5_angle = 130;
volatile long motor6_angle = 130;

// Initialzing the pin numbers on the PCA9685 Arduino Servo Driver Board that the servo motors are connected to.
uint8_t BaseServo_Pos = 0;
uint8_t Motor2_Pos = 3;
uint8_t Motor3_Pos = 4;
uint8_t Motor4_Pos = 7;
uint8_t Motor5_Pos = 12;
uint8_t Motor6_Pos = 15;

// Bluetooth control command definitions
// The control commands below control the base servo and the motor above it via the left-side joystick on the bluetooth app.
#define Base_Servo_LEFT 'L'
#define Base_Servo_RIGHT 'R'
#define MOTOR2_UP 'U'
#define MOTOR2_DN 'D'

// The control commands below control motors 3 and 4 together.
// Motor 5 which rotates the gripper left and right has seperate left/right buttons.
// These buttons are found on the right-side joystick on the bluetooth app.
#define MOTOR34_DN 'B'
#define MOTOR34_UP 'F'
#define GRIPPERMOTOR_ROTATELEFT 'S'  //// Shmel يمين
#define GRIPPERMOTOR_ROTATERIGHT 'Y' //// Yameen شمال

//// Gripper Control
#define OPEN_GRIPPER 'O' // Pressing the start button on the bluetooth app causes the gripper to open its claws gradually.
#define CLOSE_GRIPPER 'C' // Pressing the pause button on the bluetooth app causes the gripper to close its claws gradually.

int Delta_Angle = 3; // This variable represents the total change in any servo motor angle whenever it is actuated. For instance, Base_Servo_LEFT moves the base servo -3 degrees and Base_Servo_RIGHT moves the base servo +3 degrees.
int Gripper_Delta_Angle = 5; // This is a gripper delta angle just to check if the gripper acts differently than the rest of the servos.
char command; // This variable is used to save bluetooth commands.

#define Save_JointVariables_Btn 6 // We configure Digital Pin 6 of the Arduino Uno to detect a press on the push button to save the joint variables positons.
#define Execute_ActionsSequence 7 // We configure Digital Pin 7 of the Arduino Uno to detect a press on the push button to execute the sequence of saved positions.
volatile long RobotArmPositions[30][6]; // We define a 2D array for saving 30 robot arm positions (More than enough I suppose). Each position having 6 revolute joint variables. (measured in degrees)
volatile int Position_Indexer = 0;

volatile float JointAnglesDeg[6]; // To be used to copy the angles received from python into an array, which gets later send to the robotic arm joints and executed.

//// The function below is the ISR function for saving the arm positions in the 2D array, called RobotArmPositions.
void SaveArmPosition()
{
  long JointVariables[] = {base_angle, motor2_angle, motor3_angle, motor4_angle, motor5_angle, motor6_angle};
  for(int col=0; col<6; col++)
  {
    RobotArmPositions[Position_Indexer][col] = JointVariables[col]; // Save the current 6 joint variables in the array of robot arm positions. 
  }
  Position_Indexer++;
}

//// The function below is the ISR function for executing the sequence of arm positions 
void ExecuteSequence()
{
  for(int i=0; i<Position_Indexer; i++)
  {
    ActuateMotors_Positions(RobotArmPositions, i); // We execute the saved robotic arm positions one by one and in order one after the other.
    delay(2000); // Delay 2 seconds between each robot arm position execution to ensure smoothness of motion.
  }
  Position_Indexer = 0; // Reset Position_Indexer value back to 0 in order to repeat the ExecuteSequence process if needed. 
}


void setup() 
{
  // put your setup code here, to run once:
  Wire.begin();   // Initialize I2C communication between Arduino and Servo Motor Driver via SDA (to pin A4) and SCL (to pin A5) pins. A4 SDA and A5 SCL. (Ensure those connections!)
  Servos_Drive.begin();             // Start PCA9685 Servo Driver
  Servos_Drive.setOscillatorFrequency(25000000);  // 25 MHz default clock
  Servos_Drive.setPWMFreq(50);      // 50Hz is the optimal PWM frequency for the MG996R servo motors used in the robotic arm.

  BT.begin(9600);         // Initiate and begin serial communication between the arduino uno and the HC-06 Bluetooth module at a baud rate of 9600.
  Serial.begin(115200);   // Begin serial communication between the computer and arduino via RX and TX pins (0 and 1) at a baud rate of 115200. This serial port will be used for communication and interfacing with python gui.
  
  
  // We configure the button pins used to save positions and used to execute the sequence of saved positions as internally pulled up.
  // We also define the interruption cycles that they go through when a falling edge is detected.
  pinMode(Save_JointVariables_Btn, INPUT_PULLUP);
  pinMode(Execute_ActionsSequence, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(Save_JointVariables_Btn), SaveArmPosition, FALLING);
  attachInterrupt(digitalPinToInterrupt(Execute_ActionsSequence), ExecuteSequence, FALLING);

  InitializeMotor_Postions(); // This function would set the robotic arm to go back to the initial idle state.
}

void loop() 
{
  // put your main code here, to run repeatedly:
  readFromBluetooth();
  readFromPython();

}

void readFromBluetooth()
{
  // If there is something being received by the arduino from the bluetooth module, we read it and execute the corresponding action.
  if (BT.available())
  {
    command = BT.read(); // Read the bluetooth command pressed on the app and save it.
    executeBluetoothCommand(command); // This function takes the bluetooth command from the pressed button on the app and executes the corresponding motor angle change.
  }
}


// This function would accept angle values from python and execute them on the robotic arm joints.
// Angles can be given from python in radians or degrees format, it doesn't matter.
void readFromPython() 
{
  //// If there is any command received from python by the arduino controller as a string of angles, it will read the command and sort it into an array of joint values.
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n'); // Read the command received by the arduino from python via serial port and save it into the line variable.
    line.trim();

    // If the line length is less than 3, which means that no angles were provided from python to execute on the robotic arm.
    if (line.length() < 3)
    {
      Serial.print("No Joint Variable Angles were provided. No execution.");
      return;
    }
    //// Angles are received from python in this form "T,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6"

    char Angle_Check = line.charAt(0);  //// This line checks whether the angles given were in radians or in degrees. 'T' --> Radians and 'D' --> Degrees.

    line = line.substring(2); // This code removes the angle type identifier and the comma right after it. So, this would return the list of angles.

    // Split by commas
    float values[6];
    int index = 0;
    int lastComma = 0;

    for (int i = 0; i < line.length(); i++) 
    {
      if (line.charAt(i) == ',' || i == line.length() - 1) 
      {
        String valueStr;
        if (i == line.length() - 1)
          valueStr = line.substring(lastComma);
        else
          valueStr = line.substring(lastComma, i);

        values[index] = valueStr.toFloat();
        index++;

        lastComma = i + 1;

        if (index == 6) break;
      }
    }

    // If the angles were given to us from python in "Radians", we can convert it into "Degrees" using the following code.
    if (Angle_Check == 'T') 
    {
      for (int i = 0; i < 6; i++)
        values[i] = values[i] * 180.0 / PI;
    }


    // Update the joint angle variables based on the angles given by python.
    base_angle = values[0];
    motor2_angle = values[1];
    motor3_angle = values[2];
    motor4_angle = values[3];
    motor5_angle = values[4];
    motor6_angle = values[5];

    // Move the servo motors according to the angles received from python.
    Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(base_angle));
    Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(motor2_angle));
    Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(motor3_angle));
    Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(motor4_angle));
    Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(motor5_angle));
    Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(motor6_angle));

    // Send feedback to Python
    Serial.print("Joint Variable Angles Received and Executed. The angles implemented are: ");
    Serial.println(line);
  }
}


// This function takes a particular servo motor angle from 0 to 180 degrees and converts it to a PWM Pulse.
uint16_t angleToPulse(long angle) 
{
  // This function converts the Servo specified angle (between 0 and 180 degrees) to a PWM signal that can be sent to the Servo Motors for actuation.
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}


// This function below initializes the angles for all the servo motors used in the robotic arm.
void InitializeMotor_Postions()
{
  //// Initial Motor Positions for the arm to be at the center. Idle state.
  Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(95));   // Base_Servo
  Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(90));   // Motor2
  Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(100));   // Motor3
  Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(90));   // Motor4
  
  // Gripper Motors Angle Initialization Below.
  Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(130));  // Motor5 --> GripperMotor_Rotator
  Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(130));  // Motor6 --> GripperMotor_Clamper
  Serial.println("All Servos have been initialized! Robotic Arm in idle state.");

}

volatile void ActuateMotors_Positions(volatile long RobotArmPositions[30][6], int i)
{
  Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(RobotArmPositions[i][0]));     // Base_Servo
  Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(RobotArmPositions[i][1]));        // Motor2
  Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(RobotArmPositions[i][2]));        // Motor3
  Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(RobotArmPositions[i][3]));        // Motor4
  Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(RobotArmPositions[i][4]));        // Motor5 --> GripperMotor_Rotator
  Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(RobotArmPositions[i][5]));        // Motor6 --> GripperMotor_Clamper
}

void executeBluetoothCommand(char command)
{

  switch (command) 
  {
    // Move the base servo left by Delta_Angle degrees.
    case Base_Servo_LEFT:
      base_angle = constrain(base_angle + Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(base_angle));
      break;
    
    // Move the base servo right by Delta_Angle degrees.
    case Base_Servo_RIGHT:
      base_angle = constrain(base_angle - Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(base_angle));
      break;
    
    // Move the servo motor 2 up by Delta_Angle degrees.
    case MOTOR2_UP:
      motor2_angle = constrain(motor2_angle - Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(motor2_angle));
      break;
    
    // Move the servo motor 2 down by Delta_Angle degrees.
    case MOTOR2_DN:
      motor2_angle = constrain(motor2_angle + Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(motor2_angle));
      break;
    
    // Move the servo motors 3 and 4 downward together at the same time.
    case MOTOR34_DN:
      motor3_angle = constrain(motor3_angle + Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      motor4_angle = constrain(motor4_angle + Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(motor3_angle));
      Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(motor4_angle));
      break;
   
    // Move the servo motors 3 and 4 upward together at the same time.
    case MOTOR34_UP:
      motor3_angle = constrain(motor3_angle - Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      motor4_angle = constrain(motor4_angle - Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(motor3_angle));
      Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(motor4_angle));
      break;
    
    // Rotate the gripper motor leftwards.
    case GRIPPERMOTOR_ROTATELEFT:
      motor5_angle = constrain(motor5_angle - Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(motor5_angle));
      break;
    
    // Rotate the gripper motor rightwards.
    case GRIPPERMOTOR_ROTATERIGHT:
      motor5_angle = constrain(motor5_angle + Delta_Angle, 0, 180); // Makes sure that the angle is constrained between 0° and 180°.
      Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(motor5_angle));
      break;

    // Open the gripper clamps.
    case OPEN_GRIPPER:
      motor6_angle = constrain(motor6_angle - Gripper_Delta_Angle, 90, 150); // Makes sure that the angle is constrained between 90° and 150°.
      Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(motor6_angle));
      break;

    // Close the gripper clamps.
    case CLOSE_GRIPPER:
      motor6_angle = constrain(motor6_angle + Gripper_Delta_Angle, 90, 150); // Makes sure that the angle is constrained between 90° and 150°.
      Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(motor6_angle));
      break;

    default:
      // Invalid Bluetooth Command
      break;
  }

}
