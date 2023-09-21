/*---------------------------------------------------------------------------------------------

License:

Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project.

------------------------------------------------------------------------------------------------

Howdy!
Welcome to team 1977's Swerve drive code for our competiton bot in offseason 2023!
This project is based on the 2022 example swerve drive kinematics project by WPI.
This wonderful project made me extra suicidal, so be sure to enjoy it yourself!! :)

All important global variables related to robot geometry are in the Headers.h file
PDIF variables are declared in SwerveModule.h

This project was made by Salem (Me, Lead Programmer 23-24), Josh (LP 22-23), Alexandra and Dean

------------------------------------------------------------------------------------------------

Special Features:
 - Support for Per-Module centers of rotation
 - Toggle between Driver and Robot oriented drive
 - Really shitty untested autonomous capibilities
 - A gyro script that allows instancing the gyro (very cool)

------------------------------------------------------------------------------------------------

Controls:
 Joystick (Robot):
 - X/Y				:	Robot position
 - Twist			:	Robot Angle
 - Trigger			:	Fast Rotation Mode
 - Thumb Button		:	Toggle Field/Robot Oriented (Default Field)
 - Throttle			:	Robot Movement Speed (Low throttle is Low speed)

 Controller (Arm):		
 - L Trigger		:	Angle DOWN
 - R Trigger		:	Angle UP
 - L Bumper			:	Extend OUT
 - R Bumper			:	Extend IN
 - L Stick Button	:	AUTO ARM UP
 - R Stick Button	:	AUTO ARM DOWN / RETRACT
 - B Button			:	Intake IN
 - A Button			:	Intake OUT

------------------------------------------------------------------------------------------------

Robot:
 |__Drivetrain
 |  |__Swerve Module
 |  |  |__Drive Motor
 |  |  |__Angle Motor
 |  |  |__Drive Encoder (In our case this is part of the motor, as we used falcon 500s)
 |  |  |__Angle Encoder 
 |  |__Gyro
 |     |__AHRS gyro
 |__Arm
 |  |__Intake Motor
 |  |__Angle Motor
 |  |__Extension Motor
 |  |__Angle Encoder
 |  |__Angle Limit Switch
 |  |__Extension Limit Switch
 |__Vision
 |  |__Limelight 2+ (Might use raspberryPi and webcams, no vision solution currently)

-----------------------------------------------------------------------------------------------

If you have any questions you can reach me at:

	discord		:	uruplonstk
	email		:	elessem3+whiplash@gmail.com
	phone		:	(970)-825-6568
	snapchat	:	itsthatalo
	github		:	ThatAlo

THATS ALL THANKS BYE!!!

---------------------------------------------------------------------------------------------*/

#pragma once

#include "Drivetrain.h"
#include "Mechanism.h"

class Robot : public frc::TimedRobot {
	public:
		void RobotInit() override;
		void RobotPeriodic() override;

		void AutonomousInit() override;
		void AutonomousPeriodic() override;

		void TeleopInit() override;
		void TeleopPeriodic() override;

		void DisabledInit() override;
		void DisabledPeriodic() override;

		void TestInit() override;
		void TestPeriodic() override;

	private:
		frc::Joystick m_Joystick{0};

		//Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1
		frc::SlewRateLimiter<units::dimensionless::scalar> m_forwardLimiter{3 / 1_s};
		frc::SlewRateLimiter<units::dimensionless::scalar> m_strafeLimiter{3 / 1_s};
		frc::SlewRateLimiter<units::dimensionless::scalar> m_rotateLimiter{3 / 1_s};

		frc::Timer timer;

		Gyro* gyro = Gyro::GetInstance();

		Drivetrain m_swerve{};

		Mechanism m_cubeArm{};

		frc::Translation2d centerOfRotation = {0_m,0_m};
		units::degrees_per_second_t rotation = 0_deg_per_s;
	};