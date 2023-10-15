// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Drivetrain.h"
#include "Mechanism.h"
#include "Controllers.h"

class Robot : public frc::TimedRobot {
	public:
		void RobotInit() override;
		void RobotPeriodic() override;

		void ExecuteTask(double current_time, Task task);

		void AutonomousInit() override;
		void AutonomousPeriodic() override;

		void TeleopInit() override;
		void TeleopPeriodic() override;

		void DisabledInit() override;
		void DisabledPeriodic() override;

		void TestInit() override;
		void TestPeriodic() override;

	private:

		Control::Drive driver{};
		Control::Mechanism shooter{};

		cs::UsbCamera r_driveCam;
		cs::UsbCamera r_armCam;
		cs::VideoSink r_camServer;

		//Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1
		frc::SlewRateLimiter<units::dimensionless::scalar> m_forwardLimiter{3 / 1_s};
		frc::SlewRateLimiter<units::dimensionless::scalar> m_strafeLimiter{3 / 1_s};
		frc::SlewRateLimiter<units::dimensionless::scalar> m_rotateLimiter{3 / 1_s};

		frc::Timer timer;

		frc::Pose2d robot_position;

		Gyro* gyro = Gyro::GetInstance();

		Drivetrain m_swerve{};

		Mechanism m_cubeArm{};

// Auto Variables

		double threshold_angles[5] = {  30,   20,   10,    5,    2};  // Threshold angles in degrees
		double speed_multiplier[5] = {0.18, 0.08, 0.05, 0.03, 0.00};  // Associated motor speeds

// AUTO SELECTORS

		/**
		 * @param 0 Do absolutely nothing for 15 seconds
		 * @param 1 Take a wide radius around the charge station and skedaddle
		 * @param 2 Go over the charge station at a slow speed
		 * @param 3 Take a small distance to squeeze by the charge station and skedaddle.  Closest to opponent human player
		*/
		int auto_task = 0;

		/**
		 * @param UNSET Do nothing because we are not on a team
		 * @param RED We are on the RED alliance do red auto
		 * @param BLUE We are on the BLUE alliance do blue auto
		*/
		int auto_alliance = GREEN;

// Teleop variables

		frc::Translation2d center_of_rotation = {0_m,0_m};

		units::degrees_per_second_t rotation_speed = 0_deg_per_s;

		units::velocity::feet_per_second_t forward;
		units::velocity::feet_per_second_t strafe;
		units::angular_velocity::degrees_per_second_t rotate;

	};