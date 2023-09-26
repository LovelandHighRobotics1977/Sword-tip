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
	m_cubeArm.SetIntake(0,0,1,1,1);
	m_cubeArm.SetAngle(0,0);

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
}
void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
	gyro->Reset();
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
	double rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(j_rotate, 0.3)) * sqrt(throttle));

		// Determine center of rotation based on button input
	if (m_Joystick.GetRawButton(1) && !m_Joystick.GetRawButton(15)) {
		// Tower (center of mass)
		centerOfRotation = Swordtip::Frame::Center;
		rotation = rotate * Swordtip::Velocity::Rotation::Medium;
	}else if (m_Joystick.GetRawButton(1) && m_Joystick.GetRawButton(15)) {
		// Tower (center of mass)
		centerOfRotation = Swordtip::Frame::Center;
		rotation = rotate * Swordtip::Velocity::Rotation::Fast;
	} else {
		// Robot center
		centerOfRotation = Swordtip::Frame::Center;
		rotation = rotate * Swordtip::Velocity::Rotation::Slow;
	}

	// Put values to SmartDashboard
	frc::SmartDashboard::PutNumber("throttle", throttle);
	frc::SmartDashboard::PutNumber("Angle", static_cast<int>(gyro->GetYaw()));
	
	// Determine if robot is driving field oriented or robot oriented
	bool fieldOriented = !m_Joystick.GetRawButton(6);

	// Control any attached mechanisms
	if(m_Xbox.GetLeftTriggerAxis() > 0)
	{
		up = true;
	}
	else
	{
		up = false;
	}
	if (m_Xbox.GetRightTriggerAxis() > 0)
	{
		down = true;
	}
	else 
	{
		down = false;
	}

	m_cubeArm.SetAngle(up,down);
	m_cubeArm.SetIntake(m_Xbox.GetLeftBumper(), m_Xbox.GetRightBumper());
	m_cubeArm.SetSpeed(m_Xbox.GetXButton(), m_Xbox.GetYButton(), m_Xbox.GetBButton());

	// Drive the swerve modules
	if(m_Joystick.GetRawButton(5)){
		// Emergency braking (Button 5)
		m_swerve.Drive(0_mps, 0_mps, 0_deg_per_s, fieldOriented, centerOfRotation);
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