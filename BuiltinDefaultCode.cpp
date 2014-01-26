#include "WPILib.h"
#include "CANJaguar.h"
#include <Relay.h>

#include <cmath>
//#include <iostream>
using namespace std;

/****** HUGE NOTE SECTION MOVED TO BOTTOM ******/

//*****CAN ID'S USED*****//
//3, 4, 5, 6, 7, 8, 11, 18

//drive
const int FRONT_LEFT_MOTOR_ID = 6;
const int FRONT_RIGHT_MOTOR_ID = 11;
const int BACK_LEFT_MOTOR_ID = 18;
const int BACK_RIGHT_MOTOR_ID = 5;

const float SENSITIVITY = 3;

//elevator
const int ELEVATOR_MOTOR_ID = 4;
const int ELEVATOR_MOTOR2_ID = 8;

//shooter
const int FRONT_SHOOTER_ID = 3;
const int BACK_SHOOTER_ID = 7;

//*****PWM ID'S USED*****//
const int LEFT_SHIFTER_ID = 1;
const int RIGHT_SHIFTER_ID = 2;

const int LEFT_FINGER_ID = 3;
const int RIGHT_FINGER_ID = 4;

const int DRIVE_SPEED = 500;

// The Elevator positions are all based upon the voltage at the bottom position
// The bottom position has the highest voltage and everything else will be based upon that voltage
// In the case that the encoder needs to be repositioned, only the bottom voltage will need to be
// re-measured.  Everything else should stay the same
// The low, middle, and high postions will need to be measured and input as an offset from the 
// bottom voltage

const double ELEV_MAX_VOLTAGE = 5;

const double ELEV_BOTTOM_VOLTAGE = 2.005;
const double ELEV_BOTTOM_POSITION = ELEV_BOTTOM_VOLTAGE / ELEV_MAX_VOLTAGE;

const double ELEV_VOLTAGE_SPAN = .55;
// the current top position voltage is 1.485 volts
const double ELEV_TOP_VOLTAGE = ELEV_BOTTOM_VOLTAGE - ELEV_VOLTAGE_SPAN;
const double ELEV_TOP_POSITION = ELEV_TOP_VOLTAGE / ELEV_MAX_VOLTAGE;

const double DEADBAND = 0.0005;

const double AUTO_ELEV_FRONT_POSITION = ELEV_BOTTOM_POSITION - .071;
const double AUTO_ELEV_SIDE_POSITION = ELEV_BOTTOM_POSITION - .046; //.055 then .05115
const double AUTO_ELEV_REAR_POSITION = ELEV_BOTTOM_POSITION - .065;

const double AUTO_FRONT_SHOOTER_SPEED_FRONT_POSITION = 3000;
const double AUTO_BACK_SHOOTER_SPEED_FRONT_POSITION = 2900;

const double AUTO_FRONT_SHOOTER_SPEED_SIDE_POSITION = 3000;
const double AUTO_BACK_SHOOTER_SPEED_SIDE_POSITION = 2944;

const double AUTO_FRONT_SHOOTER_SPEED_REAR_POSITION = 3000;
const double AUTO_BACK_SHOOTER_SPEED_REAR_POSITION = 2960;

const double AUTO_FRONT_SHOOTER_SPEED_TOWER_POSITION = 1819;
const double AUTO_BACK_SHOOTER_SPEED_TOWER_POSITION = 1573;


const double ELEV_LOW_POSITION = ELEV_BOTTOM_POSITION - .071;

const double ELEV_MID_POSITION = ELEV_BOTTOM_POSITION - .06;

const double ELEV_HIGH_POSITION = ELEV_BOTTOM_POSITION - .090;

const double ELEV_SLOPE = ELEV_TOP_POSITION - ELEV_BOTTOM_POSITION;


const float SPEED_MULTIPLIER = .97;

//const int AVERAGE_BUFFER_SIZE = 5;

const float POT_GAIN = .01;

class BuiltinDefaultCode: public IterativeRobot {
	// There are two modes of elevator operation
	// The default is Pot control where the slider controls the eleavtor postion
	// The second is Button control where the bottons control the elevator position

	enum {
		ELEV_POT_CNTL = 0, ELEV_BUTTON_CNTL = 1
	} m_elev_cntl_mode;

	// When in button control there are four states
	// When first entering button control the elevator will stay where it was in pot control
	// until one of the buttons is pressed
	// Then it will transition to Bottom, Mid, or Top commanded mode

	enum {
		ELEV_INIT_CMD = 0,
		ELEV_LOW_CMD = 1,
		ELEV_MID_CMD = 2,
		ELEV_HIGH_CMD = 3
	} m_elev_button_mode;

	/*enum AVERAGE_INPUTS {
		ENCODER_VOLTAGE, POT_VOLTAGE, LAST = POT_VOLTAGE
	};*/

	//float averageBuffer[LAST][AVERAGE_BUFFER_SIZE];

	int autoMode;
	bool senseTF, elevatingTF, highGear, fingerOut1, fingerOut2, autoFinished;

	// Declare variable for the robot drive system

	CANJaguar *frontLeftMotor, *backLeftMotor, *frontRightMotor,
	*backRightMotor; // robot will use CAN for drive motors 1-4

	//Declare variable for the robot shoot system
	CANJaguar *FrontShooterMotor, *BackShooterMotor;

	//Declare variable for the robot elevate system
	CANJaguar *elevatorMotor, *elevatorMotor2;

	//Declare variables for the robot camera and flopper systems
	Relay *flopperSpike; //*cameraSpike, 

	// Declare a variable to use to access the driver station object
	DriverStation *m_ds; // driver station object


	// Declare variables for the two joysticks being used
	Joystick *m_rightStick; // joystick 1 (arcade stick or right tank stick)
	Joystick *m_leftStick; // joystick 2 (tank left stick, and mecanum stick)

	//Declare variable for the robot shifter system
	Servo *shifterLeft, *shifterRight/*, *fingerLeft, *fingerRight*/;

	//Declare another variable for the robot flopper system
	DigitalInput *irsensor;
	AnalogChannel *elevatorAbsoluteEncoder;

	//Timer *flopperTimer;

	static const int NUM_JOYSTICK_BUTTONS = 16;
	bool m_rightStickButtonState[(NUM_JOYSTICK_BUTTONS + 1)];
	bool m_leftStickButtonState[(NUM_JOYSTICK_BUTTONS + 1)];

	// Declare variables for each of the eight solenoid outputs
	static const int NUM_SOLENOIDS = 8;
	Solenoid *m_solenoids[(NUM_SOLENOIDS + 1)];

	// drive mode selection
	enum {
		UNINITIALIZED_DRIVE = 0, ARCADE_DRIVE = 1, TANK_DRIVE = 2
	} m_driveMode;

public:
	/**
	 * Constructor for this "BuiltinDefaultCode" Class.
	 * 
	 * The constructor creates all of the objects used for the different inputs and outputs of
	 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
	 * providing named objects for each of the robot interfaces. 
	 */
	BuiltinDefaultCode(void) {
		printf("BuiltinDefaultCode Constructor Started\n");

		// Configure CAN Drive Motors:
		frontLeftMotor = new CANJaguar(FRONT_LEFT_MOTOR_ID,
				CANJaguar::kPercentVbus); // set output from -1.0 to 1.0 (same as PWM)
		backLeftMotor = new CANJaguar(BACK_LEFT_MOTOR_ID,
				CANJaguar::kPercentVbus); // set output from -1.0 to 1.0 (same as PWM)
		frontRightMotor = new CANJaguar(FRONT_RIGHT_MOTOR_ID,
				CANJaguar::kPercentVbus); // set output from -1.0 to 1.0 (same as PWM)
		backRightMotor = new CANJaguar(BACK_RIGHT_MOTOR_ID,
				CANJaguar::kPercentVbus); // set output from -1.0 to 1.0 (same as PWM)

		frontLeftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		// set Jag to "break" when output = 0.0
		backLeftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		// set Jag to "break" when output = 0.0
		frontRightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		// set Jag to "break" when output = 0.0
		backRightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		// set Jag to "break" when output = 0.0

		frontLeftMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		frontRightMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		frontLeftMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		frontRightMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);

		frontLeftMotor->ConfigEncoderCodesPerRev(250);
		frontRightMotor->ConfigEncoderCodesPerRev(250);

		flopperSpike = new Relay(1, Relay::kBothDirections);

		irsensor = new DigitalInput(1);

		elevatorMotor = new CANJaguar(ELEVATOR_MOTOR_ID,
				CANJaguar::kPercentVbus); //Right
		elevatorMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

		elevatorAbsoluteEncoder = new AnalogChannel(1);

		elevatorMotor2 = new CANJaguar(ELEVATOR_MOTOR2_ID,
				CANJaguar::kPercentVbus); //Left
		elevatorMotor2->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		
		/*frontLeftMotor->SetPID(1,.1,0);
		frontRightMotor->SetPID(1,.1,0);*/

		FrontShooterMotor = new CANJaguar(FRONT_SHOOTER_ID, CANJaguar::kSpeed);
		FrontShooterMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		FrontShooterMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		FrontShooterMotor->ConfigEncoderCodesPerRev(250);
		FrontShooterMotor->SetPID(.4, .03, 0);
		FrontShooterMotor->EnableControl();

		BackShooterMotor = new CANJaguar(BACK_SHOOTER_ID, CANJaguar::kSpeed);
		BackShooterMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		BackShooterMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		BackShooterMotor->ConfigEncoderCodesPerRev(250);
		BackShooterMotor->SetPID(.4, .03, 0);
		BackShooterMotor->EnableControl();

		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();

		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_rightStick = new Joystick(1);
		m_leftStick = new Joystick(2);

		shifterLeft = new Servo(LEFT_SHIFTER_ID);
		shifterRight = new Servo(RIGHT_SHIFTER_ID);
		
		/*fingerLeft = new Servo(LEFT_FINGER_ID);
		fingerRight = new Servo(RIGHT_FINGER_ID);*/

		// Iterate over all the buttons on each joystick, setting state to false for each
		UINT8 buttonNum = 1; // start counting buttons at button 1
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			m_rightStickButtonState[buttonNum] = false;
			m_leftStickButtonState[buttonNum] = false;
		}

		// Iterate over all the solenoids on the robot, constructing each in turn
		UINT8 solenoidNum = 1; // start counting solenoids at solenoid 1
		for (solenoidNum = 1; solenoidNum <= NUM_SOLENOIDS; solenoidNum++) {
			m_solenoids[solenoidNum] = new Solenoid(solenoidNum);
		}

		/*for (int i = 0; i < LAST; i++) {
			for (int j = 0; j < AVERAGE_BUFFER_SIZE; j++) {
				averageBuffer[i][j] = 0;
			}
		}*/

		// Set drive mode to uninitialized
		m_driveMode = UNINITIALIZED_DRIVE;

		AxisCamera &camera = AxisCamera::GetInstance("10.26.19.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteMaxFPS(5);
		camera.WriteCompression(100);

		printf("BuiltinDefaultCode Constructor Completed\n");
	}

	/*float getAverage(AVERAGE_INPUTS input) {
		float total = 0;
		for (int i = 0; i < AVERAGE_BUFFER_SIZE; i++)
			total += averageBuffer[input][i];
		return (total / AVERAGE_BUFFER_SIZE);
	}*/

	/*void feedValue(AVERAGE_INPUTS input, float inputValue) {
		for (int i = 0; i < AVERAGE_BUFFER_SIZE - 1; i++) {
			averageBuffer[input][i] = averageBuffer[input][i + 1];
		}
		averageBuffer[input][AVERAGE_BUFFER_SIZE - 1] = inputValue;
	}*/

	void back2Feet() {
		double RPosition = (frontRightMotor->GetPosition());
		if (highGear == true) {
			while (frontRightMotor->GetPosition() < (RPosition
					+ 1.361444270015699)) {
				frontLeftMotor->Set(.8);
				backLeftMotor->Set(.8);
				backRightMotor->Set(-.8);
				frontRightMotor->Set(-.8);
				// cout<<"RightPosition:"<<frontRightMotor->GetPosition()<<endl;
			}
		} else {
			while (frontRightMotor->GetPosition() < (RPosition
					+ 2.150769230769231)) {
				frontLeftMotor->Set(.8);
				backLeftMotor->Set(.8);
				backRightMotor->Set(-.8);
				frontRightMotor->Set(-.8);
				// cout<<"RightPosition:"<<frontRightMotor->GetPosition()<<endl;
			}
		}
	}

	void spin180Degrees() {
		double LPosition = (frontLeftMotor->GetPosition());
		double RPosition = (frontRightMotor->GetPosition());
		if (highGear == true) {
			while (frontLeftMotor->GetPosition() < (LPosition + 3.0075)
					&& frontRightMotor->GetPosition() > (RPosition - 3.0075)) {
				frontLeftMotor->Set(-.8);
				backLeftMotor->Set(-.8);
				backRightMotor->Set(-.8);
				frontRightMotor->Set(-.8);
				// cout<<"LeftPosition:"<<backLeftMotor->GetPosition()<<endl<<"RightPosition:"<<frontRightMotor->GetPosition()<<endl;
			}
		} else {
			while (frontLeftMotor->GetPosition() < (LPosition + 3.33)
					&& frontRightMotor->GetPosition() > (RPosition - 3.33)) {
				frontLeftMotor->Set(-.8);
				backLeftMotor->Set(-.8);
				backRightMotor->Set(-.8);
				frontRightMotor->Set(-.8);
				// cout<<"LeftPosition:"<<backLeftMotor->GetPosition()<<endl<<"RightPosition:"<<frontRightMotor->GetPosition()<<endl;
			}
		}
	}
	void flopperMotorToggle() {

		int flopperNum = 0;

		if (m_ds->GetEnhancedIO().GetDigital(4) == false) {
			flopperSpike->Set(Relay::kForward);
			flopperNum = 1;
		} else if (m_ds ->GetEnhancedIO().GetDigital(9) == false) {
			flopperSpike->Set(Relay::kReverse);
			flopperNum = -1;
		} else if (((m_ds->GetEnhancedIO().GetDigital(4) == true)
				&& (m_ds->GetEnhancedIO().GetDigital(9) == true)) && (senseTF
						== true)) {
			flopperSpike->Set(Relay::kOff);
			flopperNum = 0;
		}

		if (m_leftStick->GetRawButton(7))
			printf("Flopper On/Off/Reverse:(1,0,-1) %i\n", flopperNum);

	}

	/*void flopperMotorAuto(bool& OnOff) {
		
		int flopperNum = 0;

		if (OnOff == false) {
			flopperSpike->Set(Relay::kForward);
			flopperNum = 1;
			OnOff = true;
		} else if ((OnOff == true) && (senseTF == true)) {
			flopperSpike->Set(Relay::kOff);
			flopperNum = 0;
			OnOff = false;
		}
		
		if (m_leftStick->GetRawButton(7))
			printf("Flopper On/Off/Reverse:(1,0,-1) %i\n", flopperNum);
		
	}*/
	
	void autonShoot() {
		
		bool OnOff = false; // With OnOff, On = true, Off = false
		for (int shooterCtr = 0; shooterCtr < 6; shooterCtr++)
		{
			if (OnOff == false) {
				flopperSpike->Set(Relay::kForward);
				OnOff = true;
			}
			Wait(.1);
			while (OnOff == true)
			{
				checkIRSensor();
				if (senseTF == true) {
					flopperSpike->Set(Relay::kOff);
					OnOff = false;
				}
			}
			Wait(1.5);
		}
		autoFinished = true;
	}
	
	void shoot() {
		double desiredSpeed = (((m_ds->GetEnhancedIO().GetAnalogIn(2) / 3.3)
				* 3100) + 900);
		double speedModify;
		const int Min_Speed = 1000;

		if (m_ds->GetEnhancedIO().GetDigital(5) == true)
			speedModify = SPEED_MULTIPLIER;
		else if (m_ds->GetEnhancedIO().GetDigital(5) == false)
			speedModify = (m_ds->GetEnhancedIO().GetAnalogIn(3))/3.3;
		
		if (desiredSpeed > 3000)
			desiredSpeed = 3000;
		else if (desiredSpeed < 900)
			desiredSpeed = 900;

		if (desiredSpeed < Min_Speed) {
			FrontShooterMotor->Set(0);
			BackShooterMotor->Set(0);
		} else if (desiredSpeed > Min_Speed) {
			FrontShooterMotor->Set(desiredSpeed);
			BackShooterMotor->Set(desiredSpeed * speedModify);
		}

		if (m_leftStick->GetRawButton(8)) {
			printf("desiredSpeed:%f\n", desiredSpeed);
			printf("encoderSpeed:%f\n", BackShooterMotor->GetSpeed());
		}
	}

	float btnFineAdjustment()
	{
		return (((m_ds->GetEnhancedIO().GetAnalogIn(1) - 1.65)/1.65)*POT_GAIN);
	}
	
	void teleopElevation() {
		int prev_elev_cntl_mode = m_elev_cntl_mode;
		double desiredPositionTele;
		if (m_ds->GetEnhancedIO().GetDigital(6) == false) {
			m_elev_cntl_mode = ELEV_POT_CNTL;
			desiredPositionTele
			= ((m_ds->GetEnhancedIO().GetAnalogIn(1)) / 3.3);
			desiredPositionTele = (desiredPositionTele * ELEV_SLOPE)
							+ ELEV_BOTTOM_POSITION;
		}

		if (m_ds->GetEnhancedIO().GetDigital(6) == true) {
			m_elev_cntl_mode = ELEV_BUTTON_CNTL;
		}

		if ((m_elev_cntl_mode == ELEV_BUTTON_CNTL) && (prev_elev_cntl_mode
				!= ELEV_BUTTON_CNTL)) {
			m_elev_button_mode = ELEV_INIT_CMD;
			desiredPositionTele
			= ((m_ds->GetEnhancedIO().GetAnalogIn(1)) / 3.3);
			desiredPositionTele = (desiredPositionTele * ELEV_SLOPE)
							+ ELEV_BOTTOM_POSITION;
		}

		if ((m_elev_cntl_mode == ELEV_BUTTON_CNTL)
				&& (m_ds->GetEnhancedIO().GetDigital(1) == false)
				|| (m_elev_button_mode == ELEV_LOW_CMD)) {
			m_elev_button_mode = ELEV_LOW_CMD;
			desiredPositionTele = ELEV_LOW_POSITION;
		}

		if ((m_elev_cntl_mode == ELEV_BUTTON_CNTL)
				&& (m_ds->GetEnhancedIO() .GetDigital(2) == false)
				|| (m_elev_button_mode == ELEV_MID_CMD)) {
			m_elev_button_mode = ELEV_MID_CMD;
			desiredPositionTele = ELEV_MID_POSITION;
		}

		if ((m_elev_cntl_mode == ELEV_BUTTON_CNTL)
				&& (m_ds->GetEnhancedIO() .GetDigital(3) == false)
				|| (m_elev_button_mode == ELEV_HIGH_CMD)) {
			m_elev_button_mode = ELEV_HIGH_CMD;
			desiredPositionTele = ELEV_HIGH_POSITION;
		}
		
		if (m_elev_cntl_mode == ELEV_BUTTON_CNTL)
		{
			desiredPositionTele-=btnFineAdjustment();
			//desiredPositionTele+=OFFSET;
		}
		
		elevate(desiredPositionTele);
	}

	void elevate(double desiredPosition) {
		if (desiredPosition > ELEV_BOTTOM_POSITION) {
			desiredPosition = ELEV_BOTTOM_POSITION;
		} else if (desiredPosition < ELEV_TOP_POSITION) {
			desiredPosition = ELEV_TOP_POSITION;
		}

		float encoderPosition = (elevatorAbsoluteEncoder->GetAverageVoltage())
						/ ELEV_MAX_VOLTAGE;

		if (desiredPosition > encoderPosition + DEADBAND) {
			elevatorMotor->Set(1);
			elevatorMotor2->Set(1);
			elevatingTF = true;
			if (m_leftStick->GetRawButton(1)) {
				printf("moving down\n");
			}
		} else if (desiredPosition < encoderPosition - DEADBAND) {
			elevatorMotor->Set(-1);
			elevatorMotor2->Set(-1);
			elevatingTF = true;
			if (m_leftStick->GetRawButton(1)) {
				printf("moving up\n");
			}
		} else {
			elevatorMotor->Set(0);
			elevatorMotor2->Set(0);
			elevatingTF = false;
		}

		if (m_leftStick->GetRawButton(9)) {
			printf("Encoder position: %f\n", encoderPosition);
			printf("Desired Position: %f \n", desiredPosition);
		}
	}

	void gearShift() {
		if (m_rightStick->GetRawButton(2)) {
			shifterRight->Set(.25);
			shifterLeft->Set(.25);
			highGear = true;
			printf("shifting up\n");
		}
		if (m_leftStick->GetRawButton(2)) {
			shifterRight->Set(.75);
			shifterLeft->Set(.75);
			highGear = false;
			printf("shifting down\n");
		}
	}
	
	/*void FingerOnOff() {
			if (m_leftStick->GetRawButton(3) && fingerOut1 == false) {
				fingerLeft->Set(.25);
				fingerRight->Set(.75);
				fingerOut1 = true;
				fingerOut2 = false;
				printf("shifting up\n");
			}
			if (m_leftStick->GetRawButton(3) && fingerOut1 == true) {
				fingerLeft->Set(.75);
				fingerRight->Set(.75);
				fingerOut1 = false;
				fingerOut2 = false;
				printf("shifting down\n");
			}
			if (m_rightStick->GetRawButton(3) && fingerOut2 == false){
				fingerLeft->Set(.75);
				fingerRight->Set(.25);
				fingerOut1 = false;
				fingerOut2 = true;
			}
			if (m_rightStick->GetRawButton(3) && fingerOut2 == true){
				fingerLeft->Set(.75);
				fingerRight->Set(.75);
				fingerOut1 = false;
				fingerOut2 = false;
			}
		}*/

	void checkIRSensor() {
		int senseData;
		senseData = irsensor->Get();
		if (senseData == 1)
			senseTF = true;
		else
			senseTF = false;

		if (m_leftStick->GetRawButton(10))
			printf("SenseData: %i \n", senseData);
	}

	void checkJags() {

		if (frontLeftMotor->GetFaults() != 0) {
			frontLeftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			frontLeftMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
			frontLeftMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
			frontLeftMotor->ConfigEncoderCodesPerRev(250);
			frontLeftMotor->EnableControl();
		}

		if (frontRightMotor->GetFaults() != 0) {
			frontRightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			frontRightMotor->SetPositionReference(
					CANJaguar::kPosRef_QuadEncoder);
			frontRightMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
			frontRightMotor->ConfigEncoderCodesPerRev(250);
			frontRightMotor->EnableControl();
		}

		if (backLeftMotor->GetFaults() != 0) {
			backLeftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			backLeftMotor->EnableControl();
		}

		if (backRightMotor->GetFaults() != 0) {
			backRightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			backRightMotor->EnableControl();
		}

		if (FrontShooterMotor->GetFaults() != 0) {
			FrontShooterMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			FrontShooterMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
			FrontShooterMotor->ConfigEncoderCodesPerRev(250);
			FrontShooterMotor->SetPID(.1, .005, 0);
			FrontShooterMotor->EnableControl();
		}

		if (BackShooterMotor->GetFaults() != 0) {
			BackShooterMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			BackShooterMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
			BackShooterMotor->ConfigEncoderCodesPerRev(250);
			BackShooterMotor->SetPID(.1, .005, 0);
			BackShooterMotor->EnableControl();
		}

		if (elevatorMotor->GetFaults() != 0) {
			elevatorMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			elevatorMotor->EnableControl();
		}

		if (elevatorMotor2->GetFaults() != 0) {
			elevatorMotor2->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			elevatorMotor2->EnableControl();
		}
	}
	void resetJags() {
		frontLeftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		backLeftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		frontRightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		backRightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

		frontLeftMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		frontRightMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);

		frontLeftMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		frontRightMotor->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);

		frontLeftMotor->ConfigEncoderCodesPerRev(250);
		frontRightMotor->ConfigEncoderCodesPerRev(250);

		elevatorMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		elevatorMotor2->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

		frontLeftMotor->EnableControl();
		backLeftMotor->EnableControl();
		frontRightMotor->EnableControl();
		backRightMotor->EnableControl();
		FrontShooterMotor->EnableControl();
		BackShooterMotor->EnableControl();
		elevatorMotor->EnableControl();
		elevatorMotor2->EnableControl();

		printf("RESET JAGS \n");
	}

	float delinearize(float x) {
		int sign = 1;
		if (x < 0) {
			sign = -1;
		}
		return sign * (atan((2 * abs(x) - 1) * (SENSITIVITY)) / atan(SENSITIVITY) + 1) / 2;
	}

	void drive(Joystick *Left, Joystick *Right) {
		float speedModifier = 1;
		if (m_rightStick->GetRawButton(4))
			speedModifier = .25;
		else if (m_rightStick->GetRawButton(1))
			speedModifier = .5;
		
		/*if (m_leftStick->GetRawButton(1))
		{
			frontLeftMotor->ChangeControlMode(CANJaguar::kSpeed);
			frontRightMotor->ChangeControlMode(CANJaguar::kSpeed);
			backLeftMotor->ChangeControlMode(CANJaguar::kVoltage);
			backRightMotor->ChangeControlMode(CANJaguar::kVoltage);
			
			frontLeftMotor->Set((delinearize(Left->GetY() * speedModifier))*DRIVE_SPEED);
			frontRightMotor->Set((delinearize(Left->GetY() * speedModifier))*DRIVE_SPEED);
			backLeftMotor->Set(frontLeftMotor->GetOutputVoltage());
			backRightMotor->Set(backRightMotor->GetOutputVoltage());
		}
		else
		{
		frontLeftMotor->ChangeControlMode(CANJaguar::kPercentVbus);
		frontRightMotor->ChangeControlMode(CANJaguar::kPercentVbus);
		backLeftMotor->ChangeControlMode(CANJaguar::kPercentVbus);
		backRightMotor->ChangeControlMode(CANJaguar::kPercentVbus);
		*/			
		if (m_driveMode == ARCADE_DRIVE) {
			//Arcade Mode
			frontLeftMotor->Set(
					(Right->GetY() * speedModifier) - (Right->GetX()
							* speedModifier));
			backLeftMotor->Set(
					(Right->GetY() * speedModifier) - (Right->GetX()
							* speedModifier));
			frontRightMotor->Set(
					(-Right->GetY() * speedModifier) - (Right ->GetX()
							* speedModifier));
			backRightMotor->Set(
					(-Right->GetY() * speedModifier) - (Right ->GetX()
							* speedModifier));
		} else if (m_driveMode == TANK_DRIVE) {
			//Tank Drive
			frontLeftMotor->Set(delinearize(Left->GetY() * speedModifier));
			backLeftMotor->Set(delinearize(Left->GetY() * speedModifier));
			frontRightMotor->Set(delinearize(-Right->GetY() * speedModifier));
			backRightMotor->Set(delinearize(-Right->GetY() * speedModifier));
		}
		//}
	}

	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		resetJags();
		printf("RobotInit() completed.\n");
	}

	void DisabledInit(void) {
		resetJags();
	}

	void AutonomousInit(void) {

		flopperSpike->Set(Relay::kOff);
		shifterRight->Set(.25);
		shifterLeft->Set(.25);
		highGear = true;

		if ((m_ds->GetEnhancedIO().GetDigital(7) == false)
				&& (m_ds ->GetEnhancedIO().GetDigital(8) == false)) {
			autoMode = 1;
		} else if ((m_ds->GetEnhancedIO().GetDigital(7) == false)
				&& (m_ds ->GetEnhancedIO().GetDigital(8) == true)) {
			autoMode = 2;
		} else if ((m_ds->GetEnhancedIO().GetDigital(7) == true)
				&& (m_ds ->GetEnhancedIO().GetDigital(8) == false)) {
			autoMode = 3;
		} else if ((m_ds->GetEnhancedIO().GetDigital(7) == true)
				&& (m_ds ->GetEnhancedIO().GetDigital(8) == true)) {
			autoMode = 4;
		}

		resetJags();
		Wait(2);

		autoFinished = false;
		
		printf("AutonomousInitComplete\n");
	}

	void TeleopInit(void) {
		m_driveMode = UNINITIALIZED_DRIVE; // Set drive mode to uninitialized
		flopperSpike->Set(Relay::kOff);
		shifterRight->Set(.25);
		shifterLeft->Set(.25);
		highGear = true;
		/*fingerRight->Set(.25);
		fingerLeft->Set(.25);*/
		fingerOut1 = false;
		fingerOut2 = false;
		resetJags();
		printf("TeleopInitComplete\n");
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {

	}

	void AutonomousPeriodic(void) {
		double desiredPositionAuto;
		checkJags();
		if (autoMode == 1) {
			desiredPositionAuto = AUTO_ELEV_FRONT_POSITION;
			FrontShooterMotor->Set(AUTO_FRONT_SHOOTER_SPEED_FRONT_POSITION);
			BackShooterMotor->Set(AUTO_BACK_SHOOTER_SPEED_FRONT_POSITION);
		} else if (autoMode == 2) {
			desiredPositionAuto = AUTO_ELEV_SIDE_POSITION;
			FrontShooterMotor->Set(AUTO_FRONT_SHOOTER_SPEED_SIDE_POSITION);
			BackShooterMotor->Set(AUTO_BACK_SHOOTER_SPEED_SIDE_POSITION);
		} else if (autoMode == 3) {
			desiredPositionAuto = AUTO_ELEV_REAR_POSITION;
			FrontShooterMotor->Set(AUTO_FRONT_SHOOTER_SPEED_REAR_POSITION);
			BackShooterMotor->Set(AUTO_BACK_SHOOTER_SPEED_REAR_POSITION);
		} else if (autoMode == 4) {
			desiredPositionAuto = .35;
			FrontShooterMotor->Set(0);
			BackShooterMotor->Set(0);
		}

		if (desiredPositionAuto > ELEV_BOTTOM_POSITION) {
			desiredPositionAuto = ELEV_BOTTOM_POSITION;
		} else if (desiredPositionAuto < ELEV_TOP_POSITION) {
			desiredPositionAuto = ELEV_TOP_POSITION;
		}

		if (autoMode != 4) {
			elevate(desiredPositionAuto);

			if (elevatingTF == false && autoFinished == false) {
				Wait(4);
				autonShoot();
			}
		} else if (autoMode == 4 && autoFinished == false) {
			flopperSpike->Set(Relay::kOff);
			elevatorMotor->Set(0);
			elevatorMotor2->Set(0);
			FrontShooterMotor->Set(0);
			BackShooterMotor->Set(0);
			back2Feet();
			spin180Degrees();
			autoFinished = true;
		}
	}

	void TeleopPeriodic(void) {
		//float inputValue;
		checkJags();
		gearShift();
		teleopElevation();
		shoot();

		/*inputValue = getAverage(POT_VOLTAGE);
		feedValue(POT_VOLTAGE, inputValue);*/

		if (m_leftStick->GetRawButton(4)) {
			spin180Degrees();
		}

		if (m_leftStick->GetRawButton(5)) {
			back2Feet();
		}

		checkIRSensor();

		flopperMotorToggle();

		if (m_rightStick->GetRawButton(8))
			resetJags();

		if ((m_rightStick->GetZ()) <= .5)
			// First 50% for Tank Mode
		{
			// use tank drive
			// Set direction of all four wheels to the defualt
			if (m_driveMode != TANK_DRIVE) {
				// if newly entered tank drive, print out a message
				printf("PlyBot Switch to Tank Drive\n");
				m_driveMode = TANK_DRIVE;
			}
		} else
			// last 50% for Arcade Drive
		{
			if (m_driveMode != ARCADE_DRIVE) {
				// if newly entered arcade drive, print out a message
				printf("PlyBot Switch to Arcade Drive\n");
				m_driveMode = ARCADE_DRIVE;
			}
		}
		drive(m_leftStick, m_rightStick);

	} // TeleopPeriodic(void)
};

START_ROBOT_CLASS(BuiltinDefaultCode)
;
/**
 * This "BuiltinDefaultCode" provides the "default code" functionality as used in the "Benchtop Test."
 * 
 *              FRC Team 2619 PlyBot Drive System Test Platform
 *              
 *  Revision: 10/7/2012 M.Rehberg       
 *      Added CANbus code.  Compiled but untested.  Need to add a test to switch between
 *      CANbus and PWM for the drive motors.  For now the PWM code is commented
 *      out using "///".
 *      Arcade Vs Tank Drive is still set via the thottle on Joystick-1 (Right).
 *      
 *  Revision: 10/18/2012 M. Rehberg
 *      Updated CANbus addressing to match Jag's on PlyBot.
 *      Compiled and loaded OK into cRIO, however the CANbus is not working.
 *      
 *  Revision: 10/19/2012   M. Rehberg
 *      Removed bad Jag (CAN-13), replaced with new Jag (CAN-16)
 *      Connected Port-1 on cRIO to 2-CAN port1
 *      Connected computer or router to 2-CAN port 2
 *      DO NOT USE cRIO PORT-2 !!!  That is not in subnet 10.26.19.0
 *      
 *      
 *              
 * 
 * The BuiltinDefaultCode extends the IterativeRobot base class to provide the "default code"
 * functionality to confirm the operation and usage of the core control system components, as 
 * used in the "Benchtop Test" described in Chapter 2 of the 2009 FRC Control System Manual.
 * 
 * This program provides features in the Disabled, Autonomous, and Teleop modes as described
 * in the benchtop test directions, including "once-a-second" debugging printouts when disabled, 
 * a "KITT light show" on the solenoid lights when in autonomous, and elementary driving
 * capabilities and "button mapping" of joysticks when teleoperated.  This demonstration
 * program also shows the use of the MotorSafety timer.
 * 
 * This demonstration is not intended to serve as a "starting template" for development of
 * robot code for a team, as there are better templates and examples created specifically
 * for that purpose.  However, teams may find the techniques used in this program to be
 * interesting possibilities for use in their own robot code.
 * 
 * The details of the behavior provided by this demonstration are summarized below:
 *  
 * Disabled Mode:
 * - Once per second, print (on the console) the number of seconds the robot has been disabled.
 * 
 * Autonomous Mode:
 * - Flash the solenoid lights like KITT in Knight Rider
 * - Example code (commented out by default) to drive forward at half-speed for 2 seconds
 * 
 * Teleop Mode:
 * - Select between three different drive options depending upon Z-location of Joystick1 (Right Stick)
 * - When "Z-Up" (on Joystick1) provide "arcade drive" on Joystick1 (Right Stick)
 * - When "Z-Middle" (on Joystick1) provide "tank drive" on Joystick1 and Joystick2
 * - When "Z-Down"   (on Joystick2, Logitec Extream Pro 3D) provide "mecanum drive" on Left Stick
 * - Use Joystick buttons (on Joystick1 or Joystick2) to display the button number in binary on
 *   the solenoid LEDs (Note that this feature can be used to easily "map out" the buttons on a
 *   Joystick.  Note also that if multiple buttons are pressed simultaneously, a "15" is displayed
 *   on the solenoid LEDs to indicate that multiple buttons are pressed.)
 *
 * This code assumes the following connections:
 * - Driver Station:
 *   - USB 1 - The "right" joystick(1)  Used for either "arcade drive" or "right" stick for tank drive
 *                                                                                Also used to set drive mode using "throttle"
 *   - USB 2 - The "left" joystick(2)   Used as the "left" stick for tank drive
 *                                        This joystick needs to have "twist" input for Mecanum drive
 * 
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1/3 - Connected to "left" drive motor(s)  (front/rear)
 *     - PWM 2/4 - Connected to "right" drive motor(s) (front/rear)
 */
