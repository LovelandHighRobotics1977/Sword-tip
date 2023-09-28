// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
	frc::SmartDashboard::PutString("Match State","   Disabled");
	frc::SmartDashboard::PutString("Robot Name",Swordtip::Robot_Name);
	frc::SmartDashboard::PutNumber("X_POS", 0);
	frc::SmartDashboard::PutNumber("Y_POS", 0);

	frc::SmartDashboard::PutNumber("X_VEL", 0);
	frc::SmartDashboard::PutNumber("Y_VEL", 0);

	m_swerve.Drive(0_fps,0_fps,0_deg_per_s,0,Swordtip::Frame::Center);
	m_cubeArm.SetIntake(0,0);
	m_cubeArm.SetAngle(0,0);
	m_cubeArm.SetSpeed(0,0,0);

	timer.Start();

	/*
	r_driveCam = frc::CameraServer::StartAutomaticCapture(0);
	r_armCam = frc::CameraServer::StartAutomaticCapture(1);

	r_driveCam.SetPixelFormat(cs::VideoMode::PixelFormat::kYUYV);
	r_armCam.SetPixelFormat(cs::VideoMode::PixelFormat::kYUYV);
	*/
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	frc::SmartDashboard::PutString("Match State","   Autonomous");
	m_swerve.Drive(0_mps,0_mps,0_deg_per_s,0,Swordtip::Frame::Center);
	m_swerve.SetNeutralMode(Brake);
	timer.Start();
	timer.Restart();
	gyro->Reset();
}
void Robot::AutonomousPeriodic() {

	int currentTime = std::ceil(timer.Get().value());

	// Set autonomous stage based on time elapsed
	switch (currentTime){
		case 1: auto_stage = 1; break;
		case 2: auto_stage = 2; break;
		case 3: auto_stage = 3; break;
		case 4: auto_stage = 4; break;
		default: auto_stage = 4; break;
	}

	// Autonomous stages
	switch (auto_stage){
		case 1: 
			// Fire a cube into the mid node
			m_cubeArm.SetSpeed(0,1,0);
			m_cubeArm.SetIntake(0,1);
			break;

		case 2: 
			// Stop the intake and drive quickly away going slightly to the right
			m_cubeArm.SetIntake(0,0);
			m_swerve.Drive(Swordtip::Velocity::Maximums::Max_Speed,0.2_fps,0_deg_per_s,1,Swordtip::Frame::Center);
			break;

		case 3: 
			// Drive to the left to approach the charge station from behind
			m_swerve.Drive(0_mps,0_mps,90_deg_per_s,1,Swordtip::Frame::Center);
			break;

		case 4:
			// Auto Balance the robot
			m_swerve.AutoBalance();
			break;

		default:
			// Freeze the robot if not in any stage
			m_swerve.HaltRobot();
			break;

	}
}

void Robot::TeleopInit() {
	gyro->Reset();
	m_swerve.SetNeutralMode(Brake);
	frc::SmartDashboard::PutString("Match State","   TeleOperated");
}

void Robot::TeleopPeriodic() {
	// Reset Gyro
	if(m_Joystick.GetRawButton(2)){
		gyro->Reset();
	}

	// Get joystick inputs and calculate throttle
	double j_forward = m_Joystick.GetY();
	double j_strafe = -m_Joystick.GetX();
	double j_rotate = m_Joystick.GetRawAxis(5);
	double throttle = ((1 - ((m_Joystick.GetZ() + 1) / 2)));

	// Calculate swerve module inputs
	auto forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(j_forward, 0.2)) * throttle) * Swordtip::Velocity::Maximums::Max_Speed;
	auto strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(j_strafe, 0.2)) * throttle) * Swordtip::Velocity::Maximums::Max_Speed;
	auto rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(j_rotate, 0.3)) * sqrt(throttle)) * Swordtip::Velocity::Maximums::Max_Rotation;

	// Determine center of rotation based on button input
	if (m_Joystick.GetRawButton(1) && !m_Joystick.GetRawButton(15)) {
		// Tower (center of mass)
		centerOfRotation = Swordtip::Frame::Center;
		rotation = rotate / 2;
	}else if (m_Joystick.GetRawButton(1) && m_Joystick.GetRawButton(15)) {
		// Tower (center of mass)
		centerOfRotation = Swordtip::Frame::Center;
		rotation = rotate / 1;
	} else {
		// Robot center
		centerOfRotation = Swordtip::Frame::Center;
		rotation = rotate / 3;
	}

	// Put values to SmartDashboard
	frc::SmartDashboard::PutNumber("throttle", throttle);
	frc::SmartDashboard::PutNumber("Angle", static_cast<int>(gyro->GetYaw()));
	
	// Determine if robot is driving field oriented or robot oriented
	bool fieldOriented = !m_Joystick.GetRawButton(6);

	// Control any attached mechanisms
	m_cubeArm.SetAngle(m_Xbox.GetLeftTriggerAxis() > 0,m_Xbox.GetRightTriggerAxis() > 0);
	m_cubeArm.SetIntake(m_Xbox.GetLeftBumper(), m_Xbox.GetRightBumper());
	m_cubeArm.SetSpeed(m_Xbox.GetXButton(), m_Xbox.GetYButton(), m_Xbox.GetBButton());
	
	// Drive the swerve modules
	if(m_Joystick.GetRawButton(5)){
		// Emergency braking (Button 5)
		m_swerve.HaltRobot();
	}else{
		// Normal Drive
		m_swerve.Drive(forward, strafe, rotation, fieldOriented, centerOfRotation);
	}

	auto robot_position = m_swerve.UpdateOdometry();

	frc::SmartDashboard::PutNumber("X_POS", robot_position.X().value());
	frc::SmartDashboard::PutNumber("Y_POS", robot_position.Y().value());

	frc::SmartDashboard::PutNumber("X_VEL", gyro->GetVelocityX());
	frc::SmartDashboard::PutNumber("Y_VEL", gyro->GetVelocityY());
}

void Robot::DisabledInit() {
	frc::SmartDashboard::PutString("Match State","   Disabled");
	m_swerve.HaltRobot();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
	frc::SmartDashboard::PutString("Match State","   Test");
}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRc_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif