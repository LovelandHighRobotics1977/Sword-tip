// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
	frc::SmartDashboard::PutNumber("vD:",0);
	frc::SmartDashboard::PutNumber("vX:",0);
	frc::SmartDashboard::PutNumber("vY:",0);
	frc::SmartDashboard::PutNumber("Max Speed",1);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
}
void Robot::AutonomousPeriodic() {
	auto angle = units::degrees_per_second_t{frc::SmartDashboard::GetNumber("vD:",0)};
	auto vx = units::meters_per_second_t{-frc::SmartDashboard::GetNumber("vX:",0)};
	auto vy = units::meters_per_second_t{frc::SmartDashboard::GetNumber("vY:",0)};
	
	m_swerve.Drive(-vx,-vy,angle,1,{0_m,0_m});
}

void Robot::TeleopInit() {
	gyro->Reset();
}
void Robot::TeleopPeriodic() {
	// Reset Gyro
	if(m_Joystick.GetPOV() != -1){
		gyro->Reset();
	}

	// Get joystick inputs and calculate throttle
	double j_forward = m_Joystick.GetY();
	double j_strafe = -m_Joystick.GetX();
	double j_rotate = m_Joystick.GetTwist();
	double throttle = ((1 - ((m_Joystick.GetThrottle() + 1) / 2)) * frc::SmartDashboard::GetNumber("Max Speed", 1));

	// Calculate swerve module inputs
	const auto forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(j_forward, 0.2)) * throttle) * kMaxSpeed;
	const auto strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(j_strafe, 0.2)) * throttle) * kMaxSpeed;
	const auto rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(j_rotate, 0.3)) * sqrt(throttle));

	// Determine center of rotation based on button input
	int rotateAtPosition[1] {m_Joystick.GetRawButton(1)};
	auto [t] = rotateAtPosition;
	if (t > 0) {
		// Tower (center of mass)
		centerOfRotation = kCenterOfMass;
		rotation = rotate * kFastRotation;
	} else {
		// Robot center
		centerOfRotation = kCenterOfRobot;
		rotation = rotate * kMediumRotation;
	}

	// Put values to SmartDashboard
	frc::SmartDashboard::PutNumber("throttle", throttle);
	frc::SmartDashboard::PutNumber("Angle", static_cast<int>(gyro->GetYaw()));
	
	// Determine if robot is driving field oriented or robot oriented
	bool fieldOriented = !m_Joystick.GetRawButton(2);

	// Control any attached mechanisms
	m_somethingMachine.DoSomething(m_Joystick.GetRawButton(5));

	// Drive the swerve modules
	if(m_Joystick.GetRawButton(4)){
		// Emergency braking (Button 4)
		m_swerve.Drive(0_mps, 0_mps, 0_deg_per_s, fieldOriented, centerOfRotation);
	}else if(m_Joystick.GetRawButton(3)){
		// No rotation Drive
		m_swerve.Drive(forward, strafe, 0_deg_per_s, fieldOriented, centerOfRotation);
	}else{
		// Normal Drive
		m_swerve.Drive(forward, strafe, rotation, fieldOriented, centerOfRotation);
	}
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRc_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif