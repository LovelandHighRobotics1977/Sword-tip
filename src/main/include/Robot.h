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

		// THE IMPORTANT ONE

		int auto_mode = 3;

// Teleop variables

		frc::Translation2d center_of_rotation = {0_m,0_m};

		units::degrees_per_second_t rotation_speed = 0_deg_per_s;

		units::velocity::feet_per_second_t forward;
		units::velocity::feet_per_second_t strafe;
		units::angular_velocity::degrees_per_second_t rotate;

	};