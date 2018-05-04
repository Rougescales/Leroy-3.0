
#pragma config(Sensor, in1,    leftDR4BPot,    sensorPotentiometer)
#pragma config(Sensor, in2,    rightDR4BPot,   sensorPotentiometer)
#pragma config(Sensor, dgtl1,  ,               sensorTouch)
#pragma config(Sensor, dgtl2,  moGoBackLimitRight, sensorTouch)
#pragma config(Sensor, dgtl3,  leftDriveEncoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  rightDriveEncoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  armEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  rollerIntakeLimit,     sensorTouch)
#pragma config(Sensor, dgtl10, moGoBackLimitLeft, sensorTouch)
#pragma config(Motor,  port1,           rightMoGo,     tmotorVex393_HBridge, openLoop, driveLeft)
#pragma config(Motor,  port2,           rightDrive1,   tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port3,           rightDrive2,   tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port4,           rightDR4B,     tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port5,           arm,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           leftDR4B,      tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port8,           leftDrive2,    tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port9,           leftDrive1,    tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port10,          leftMoGo,      tmotorVex393_HBridge, openLoop, reversed, driveRight)
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"



/*------------------------------------------------------------------------------*/
/*                          PID DriveBase Control Task                          */
/*  This is a custom made task, specificially to control the drive base         */
/*  of the robot. Its intention is to make it easier to program an autonomous   */
/*  as the robot will now be able to drive straight using PID.                  */
/*  This task should be terminated upon completion of autonomous in order       */
/*  to not waste cpu time                                                       */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                                   Tuning                                     */
/*																																						  */
/*  1) Set Kp Ki and Kd to 0 (disable them).                                    */
/*																																						  */
/*  2) Increase Kp until the error is fairly small, but it still gets from the  */
/*     beginning to nearly the end quickly enough.													    */
/*																																						  */
/*  3) Increase Kd until any overshoot you may have is fairly minimal. But be   */
/*     careful with Kd - too much will make it overshoot.                       */
/*																																						  */
/*  4) Increase Ki until any error that is still existing is elimnated. Start   */
/*     with a really small number for Ki, dont be surprised if it is as small   */
/*     0.0001 or even smaller.																								  */
/*																																						  */
/*  5) Using the rules of tuning the constants (in the table below), you can    */
/*     change around the constants a little bit to get it working to the best   */
/*     performance.																														  */
/*																																						  */
/*  Rise time - the time it takes to get from the beginning point to the        */
/*  target point.																															  */
/*																																						  */
/*  Overshoot - the amount that is changed too much; the value further than     */
/*  error.																																		  */
/*																																						  */
/*  Settling time - the time it takes to settle back down when encountering a   */
/*  change.																																		  */
/*																																						  */
/*  Steady-state error - the error at the equilibrium.												  */
/*																																						  */
/*  Stability - the "smoothness" of the speed.																  */
/*																																						  */
/*																																						  */
/*            What happens when each of the constants is increased:             */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Constant: |Rise Time: | Overshoot: |Settling    |Steady-state |Stability: | */
/* |			 	  |					  |					   |time:	      |error:			  |					  | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Kp        |Decrease   |Increase    |Small change|Decrease     |Degrade    | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Ki        |Decrease   |Increase    |Increase    |Decrease     |Degrade    | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Kd        |Minor      |Decrease    |Decrease    |No effect    |Improve    | */
/* |          |change     |            |            |             |(if small  | */
/* |          |           |            |            |             |  enough)  | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/*																																							*/
/*------------------------------------------------------------------------------*/

static float DriveKp = 0.3;
static float DriveKi = 0.0001;
static float DriveKd = 0.1;
static float DriveTargetLeft;
static float DriveTargetRight;

task DrivePID()
{
	/*VARIABLE DECLARATION*/
	float DriveSensorLeft;
	float DriveSensorRight;
	float DriveIntegralLeft = 0;
	float DriveIntegralRight = 0;
	float DriveErrorLeft;
	float DriveErrorRight;
	float DriveDerivativeLeft;
	float DriveDerivativeRight;
	float DrivePreviousErrorLeft = 0;
	float DrivePreviousErrorRight = 0;
	float DriveSpeedRight;
	float DriveSpeedLeft;

	while(true)
	{
		DriveSensorLeft = SensorValue[leftDriveEncoder];//Current Left SensorValue
		DriveSensorRight = SensorValue[rightDriveEncoder];//Current Right SensorValue
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		DriveErrorLeft = DriveTargetLeft - DriveSensorLeft;//Calculate Left Error
		DriveErrorRight = DriveTargetRight - DriveSensorRight;// Calculate Right Error
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		DriveIntegralLeft = DriveIntegralLeft + DriveErrorLeft;//Calculate Left Integral
		DriveIntegralRight = DriveIntegralRight + DriveErrorRight;//Calculate Right Integral
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		DriveDerivativeLeft = DriveErrorLeft - DrivePreviousErrorLeft;//Calculate Left Derivative
		DriveDerivativeRight = DriveErrorRight -  DrivePreviousErrorRight;//Calculate Right Derivative
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		DrivePreviousErrorLeft = DriveErrorLeft;//Calculate Left Previous Error and New Error
		DrivePreviousErrorRight = DriveErrorRight;//Calculate Right Previous Error and New Error
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		if ((DriveErrorLeft) == 0) //If there is no error, No integrals shall be there because speed = kp*error + integral
		{
			DriveIntegralLeft = 0;
		}
		if((DriveErrorRight) == 0)//If there is no error, no integrals shall be ther ebecause seed = kp*error + integral
		{
			DriveIntegralRight  =0;
		}
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		if(abs(DriveErrorLeft) > 38)//Placeholder
		{
			DriveIntegralLeft = 0;//To stop the Overshoot from happening too much
		}
		if(abs(DriveErrorRight) > 38)//Placeholder
		{
			DriveIntegralRight = 0;//To stop the overshoot from happening too much
		}
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		DriveSpeedLeft = (DriveKp*DriveErrorLeft) + (DriveKi*DriveIntegralLeft) + (DriveKd*DriveDerivativeLeft);//Left speed calculation
		DriveSpeedRight = (DriveKp*DriveErrorRight) + (DriveKi*DriveIntegralRight) + (DriveKd*DriveDerivativeRight);//Right speed calculation
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//left motor power is within 90 in order to make the motor function properly
		if(DriveSpeedLeft > 90)
		{
			DriveSpeedLeft = 90;
		}
		else if(DriveSpeedLeft < -90)
		{
			DriveSpeedLeft = -90;
		}
		//right motor power is within 90 in order to make the motor function properly
		if(DriveSpeedRight > 90)
		{
			DriveSpeedLeft = 90;
		}
		else if (DriveSpeedLeft < -90)
		{
			DriveSpeedLeft = -90;
		}
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//Send to Motors
		motor[leftDrive1] = DriveSpeedLeft;
		motor[leftDrive2] = DriveSpeedLeft;
		motor[rightDrive1] = DriveSpeedRight;
		motor[rightDrive2] = DriveSpeedRight;
		/*--------------------------------------------------------------------------------------------------------------------------------*/
	}

}


/*------------------------------------------------------------------------------*/
/*                            PID DR4B Control Task                             */
/*  This is a custom made task, specificially to control the DR4B               */
/*  of the robot. Its intention is to make it easier to control the DR4B        */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                                   Tuning                                     */
/*																																						  */
/*  1) Set Kp Ki and Kd to 0 (disable them).                                    */
/*																																						  */
/*  2) Increase Kp until the error is fairly small, but it still gets from the  */
/*     beginning to nearly the end quickly enough.													    */
/*																																						  */
/*  3) Increase Kd until any overshoot you may have is fairly minimal. But be   */
/*     careful with Kd - too much will make it overshoot.                       */
/*																																						  */
/*  4) Increase Ki until any error that is still existing is elimnated. Start   */
/*     with a really small number for Ki, dont be surprised if it is as small   */
/*     0.0001 or even smaller.																								  */
/*																																						  */
/*  5) Using the rules of tuning the constants (in the table below), you can    */
/*     change around the constants a little bit to get it working to the best   */
/*     performance.																														  */
/*																																						  */
/*  Rise time - the time it takes to get from the beginning point to the        */
/*  target point.																															  */
/*																																						  */
/*  Overshoot - the amount that is changed too much; the value further than     */
/*  error.																																		  */
/*																																						  */
/*  Settling time - the time it takes to settle back down when encountering a   */
/*  change.																																		  */
/*																																						  */
/*  Steady-state error - the error at the equilibrium.												  */
/*																																						  */
/*  Stability - the "smoothness" of the speed.																  */
/*																																						  */
/*																																						  */
/*            What happens when each of the constants is increased:             */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Constant: |Rise Time: | Overshoot: |Settling    |Steady-state |Stability: | */
/* |			 	  |					  |					   |time:	      |error:			  |					  | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Kp        |Decrease   |Increase    |Small change|Decrease     |Degrade    | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Ki        |Decrease   |Increase    |Increase    |Decrease     |Degrade    | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/* |Kd        |Minor      |Decrease    |Decrease    |No effect    |Improve    | */
/* |          |change     |            |            |             |(if small  | */
/* |          |           |            |            |             |  enough)  | */
/* |----------|-----------|------------|------------|-------------|-----------| */
/*																																							*/
/*------------------------------------------------------------------------------*/
static float DR4BKp = 0.2;//Placeholder or 0.4 idk what should i do i suck at coding help puttttttttt
static float DR4BKi = 0.03;//Placeholder
static float DR4BKd = 1.5;//Placeholder

static float DR4BTargetLeft; // Used outside the PID task & inside the task
static float DR4BTargetRight;

task DR4BPID()
{
	/*--------------------------------------------------------------------------------------------------------------------------------*/
	/*VARIABLE DECLARATION*/
	float DR4BSensorLeft;
	float DR4BSensorRight;
	float DR4BIntegralLeft = 0;
	float DR4BIntegralRight = 0;
	float DR4BErrorLeft;
	float DR4BErrorRight;
	float DR4BDerivativeLeft;
	float DR4BDerivativeRight;
	float DR4BPreviousErrorLeft = 0;
	float DR4BPreviousErrorRight = 0;
	float DR4BSpeedRight;
	float DR4BSpeedLeft;
	/*--------------------------------------------------------------------------------------------------------------------------------*/



	while(true)
	{
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//Read Sensors
		DR4BSensorLeft = SensorValue[leftDR4BPot];
		DR4BSensorRight = SensorValue[rightDR4BPot];
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		// Calculate Error
		DR4BErrorLeft = DR4BTargetLeft - DR4BSensorLeft;
		DR4BErrorRight = DR4BTargetRight - DR4BSensorRight;
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		// Calculate Integral
		DR4BIntegralLeft = DR4BIntegralLeft + DR4BErrorLeft;
		DR4BIntegralRight = DR4BIntegralRight + DR4BErrorRight;
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		// Calculate Derivative
		DR4BDerivativeLeft = DR4BErrorLeft - DR4BPreviousErrorLeft;
		DR4BDerivativeRight = DR4BErrorRight -  DR4BPreviousErrorRight;
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//Calculate Previous Error and New Error
		DR4BPreviousErrorLeft = DR4BErrorLeft;
		DR4BPreviousErrorRight = DR4BErrorRight;
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//Integral adjustments due to no errors
		if ((DR4BErrorLeft) == 0)
		{
			DR4BIntegralLeft = 0;
		}

		if((DR4BErrorRight) == 0)
		{
			DR4BIntegralRight  =0;
		}
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//Integral adjustments due to too much errors
		if(abs(DR4BErrorLeft) > 15)//Placeholder
		{
			DR4BIntegralLeft = 0;
		}
		if(abs(DR4BErrorRight) > 15)//Placeholder
		{
			DR4BIntegralRight = 0;
		}
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		// Speed calcualation(Final PID Calculation)
		DR4BSpeedLeft = (DR4BKp*DR4BErrorLeft) + (DR4BKi*DR4BIntegralLeft) + (DR4BKd*DR4BDerivativeLeft);//Left speed calculation
		DR4BSpeedRight = (DR4BKp*DR4BErrorRight) + (DR4BKi*DR4BIntegralRight) + (DR4BKd*DR4BDerivativeRight);//Right speed calculation
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//Limit the speed to a reasonable number
		if(DR4BSpeedLeft > 90)
		{
			DR4BSpeedLeft = 90;
		}
		else if(DR4BSpeedLeft < -90)
		{
			DR4BSpeedLeft = -90;
		}
		if(DR4BSpeedRight > 90)
		{
			DR4BSpeedRight = 90;
		}
		if(DR4BSpeedRight < -90)
		{
			DR4BSpeedRight = -90;
		}
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		//Assign PID code to motors
		motor[leftDR4B] = DR4BSpeedLeft;
		motor[rightDR4B] = DR4BSpeedRight;
		/*--------------------------------------------------------------------------------------------------------------------------------*/
	}

}


/*Here below are the voids for all the functions

/*--------------------------------------------------------------------------------------------------------------------------------*/
// A void for right side base driving
void rightDrive (int rightDrive)
{
	if(abs(rightDrive) >= 10)
	{
		motor[rightDrive1] = rightDrive;
		motor[rightDrive2] = rightDrive;
	}
	else
	{
		motor[rightDrive1] = 0;
		motor[rightDrive2] = 0;
	}
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
//void for left drive
void leftDrive (int leftDrive)
{
	if(abs(leftDrive) >= 10)
	{
		motor[leftDrive1] = leftDrive;
		motor[leftDrive2] = leftDrive;
	}
	else
	{
		motor[leftDrive1] = 0;
		motor[leftDrive2] = 0;
	}
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
//void for DR4B controls (Only for autonomous)
void DR4B(int down, int up)
{
	if(down == 1)
	{
		motor[leftDR4B] = -127;
		motor[rightDR4B] = -127;
	}
	if(up == 1)
	{
		motor[leftDR4B] = 127;
		motor[rightDR4B] = 127;
	}
	else
	{
		motor[leftDR4B] = 0;
		motor[rightDR4B] = 0;
	}
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
//Void for MOGO lift
void moGo (int up, int down)
{
	if (up == 1)
	{
		if (SensorValue (moGoBackLimitLeft) == 0 && SensorValue (moGoBackLimitRight) == 0)
		{
			motor[leftMoGo] = 90;
			motor[rightMoGo] = 90;
		}
		else if (SensorValue(moGoBackLimitLeft) == 1 && SensorValue (moGoBackLimitRight) == 1)
		{
			motor[leftMoGo] = 0;
			motor[rightMoGo] = 0;
		}
	}
	if (down == 1)
	{
		motor[leftMoGo] = -90;
		motor[rightMoGo] = -90;
	}
	else
	{
		motor[leftMoGo] = 0;
		motor[rightMoGo] = 0;
	}
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
//Void for arm (Used in stacking void below)
void armControl (int down, int up)
{
	if(up == 1)
	{
		motor[arm] = 90;
	}
	else if(down == 1)
	{
		motor[arm] = 90;
	}
	else
	{
		motor[arm] = 0;
	}
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
//void for stacking (Includes Rollers and arm)
void stacking (int up )
{
	int rollerState = 0;
	int armState = 0;
	int somenumber = 1;

	if (up == 1)
	{
		startTask(DR4BPID);
		DR4BTargetRight = somenumber;//placeholder
		DR4BTargetLeft = somenumber;//placeholder
		while(up == 1)
		{
			if(SensorValue(armEncoder) < somenumber && rollerState == 1) // if armEncoder is less than a specified number, move the arm down until the encoder reads greater than that number
			{
				armControl(1,0);//arm extends to cone grabbing position
				armState = 1;
			}
			else if(SensorValue(armEncoder) > somenumber && rollerState == 2)//if the armEncoder is greater than a specified number, (which it would be once the arm is down)
			{ 																							//and a cone is detected inside the roller intake, move the arm back up until the roller intake is above the mobile goal
				armControl(0,1);
				armState = 2;
			}
			else if(SensorValue(armEncoder) < somenumber && SensorValue(armEncoder) > somenumber && rollerState == 2 && armState == 2) //If arm is in a specific position, has already completed the previous 2
			{ //stages of the program, and is holding a cone,
				armControl(0,0);//stop the arm
				rollerState = 3; //tell the roller to release the cone
			}
			else
			{
				armControl(0,0);//stop the arm if all the requirements are met
			}
			if(SensorValue(rollerIntakeLimit) == 0 && rollerState != 3)
			{
				motor[claw] = 63; //if the roller intake is not detecting a cone, keep trying to pickup a cone,
				rollerState = 1; //activate the first stage, telling the arm to move down
			}
			else if(SensorValue(rollerIntakeLimit) == 1 && rollerState !=3)
			{
				motor[claw] = 0;
				rollerState = 2; //if the roller intake detects a cone, stop the intake, activate the second stage, telling the arm to move back up.
			}
			else if(rollerState == 3)
			{
				motor[claw] = -63;//Release the cone
			}
		}
		motor[claw] = 0;
	}
}
/*--------------------------------------------------------------------------------------------------------------------------------*/

void rollers (int in,int out)
{
	if (in == 1)
	{
		motor[claw] = 90;
	}
	else if (out == 1)
	{
		motor[claw] = -90;
	}
	else
	{
		motor[claw] = 0;
	}
}



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void auton1()
{
	/*
	1 = to side moGo
	2 = 90 degrees turn
	3 = 20 pts to parking
	*/
	stacking(1);//preload
	DriveTargetLeft = 1;
	DriveTargetRight = 1;
	moGo(1,0);
	stacking(1);
	stacking(1);
	DriveTargetLeft = 2;
	DriveTargetRight = -2;
	DriveTargetLeft = 1;
	DriveTargetRight = 1;
	moGo(0,1);
	DriveTargetLeft = 3;
	DriveTargetRight = 3;



}
task autonomous()
{

	// ..........................................................................
	// Insert user code here.
	// ..........................................................................

	// Remove this function call once you have "real" code.
	AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	// User control code here, inside the loop

	while (true)
	{
		static bool PartnerCtrl;
		bool partnerjoystick = false;
		bool buttonPressed = false;
		/*Code reading guide:
		1. vexRT means remote control
		2. Btn??Xmtr means partner remote
		/*--------------------------------------------------------------------------------------------------------------------------------*/

		if(PartnerCtrl == 1)
		{
			if (vexRT[Btn])
		}

		/*	DR4B(1,0);
		wait1Msec(1000);
		float offsetLeft = SensorValue[leftDR4BPot];
		float offsetRight = SensorValue[rightDR4BPot];
		float wantedHeight = 0;
		float restHeight = 0;
		float coneHeight = 25; //needs to be set
		float stationaryGoalHeight = 500; //needs to be set
		float maxHeight = 950; //needs to be set
		float manualcontrol;
		DR4BTargetLeft = SensorValue[leftDR4BPot];
		DR4BTargetRight = SensorValue[rightDR4BPot];
		startTask(DR4BPID);

		int DR4BcontrolState = 0;
		*/
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		while (true)
		{

			/*NOTES:
			Main controller is Stacking and DR4B controls
			Partner is Drive and MOGO
			*/
			/*--------------------------------------------------------------------------------------------------------------------------------*/
			//DR4B
			float offsetLeft = SensorValue[leftDR4BPot];
		  float offsetRight = SensorValue[rightDR4BPot];
			float wantedHeight = 0;
			const float restHeight = 0;
		  float coneHeight = 25; //needs to be set
			const float stationaryGoalHeight = 500; //needs to be set
			const float maxHeight = 950; //needs to be set
			startTask(DR4BPID);
			if(vexRT(Btn7U) == 1)//DR4B moves up 1 cone height
			{
			wantedHeight = coneHeight;
			coneHeight = coneHeight + 1;
			}
			if(vexRT(Btn7D) == 1)//DR4B moves down 1 cone height
			{
			wantedHeight = coneHeight - 1;
			}
			else if(vexRT(Btn7L) == 1)//DR4B moves to stationary goal height
			{
			wantedHeight = stationaryGoalHeight;
			}
			else if(vexRT(Btn7R) == 1)//DR4B moves all the way down
			{
			wantedHeight = 0;
			}
			else if(vexRT(Btn5U) == 1)//Calculation of offset and wanted height making Target height
			{
			DR4BTargetLeft = wantedHeight + offsetLeft;
			DR4BTargetRight = wantedHeight + offsetRight;
			}
			if(vexRT(Btn8R) == 1)//Recalibrate offset for when Potentiometer malfunctions
			{
			offsetLeft = SensorValue(leftDR4BPot);
			offsetRight = SensorValue(rightDR4BPot);
			}
			/*--------------------------------------------------------------------------------------------------------------------------------*/
			if(vexRT[Btn8R]%2 == 1)
			{
				PartnerCtrl = true;
			}
			else if(vexRT[Btn8R]%2 == 0)
			{
				PartnerCtrl = false;
			}

			//Drive
			rightDrive(vexRT(Ch2Xmtr2));//Right side drive responds to right joystick (PID code not used here)
			leftDrive(vexRT(Ch3Xmtr2));//Left side driving responds to left joystick (PID code not used here)
			//moGO
			moGo(vexRT[Btn6UXmtr2],vexRT[Btn6DXmtr2]);
			//arm
			armControl(vexRT[Btn7DXmtr2],vexRT[Btn7UXmtr2]);
			//rollers
			rollers(vexRT[Btn7LXmtr2],vexRT[Btn7RXmtr2]);


	}
}
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
