// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::Drive(units::meters_per_second_t forward, units::meters_per_second_t strafe, units::degrees_per_second_t rotate, bool fieldRelative, frc::Translation2d centerOfRotation) {
	auto states = m_kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
													frc::ChassisSpeeds{forward, strafe, rotate}, gyro->GetRotation2d()) 
												  : frc::ChassisSpeeds{forward, strafe, rotate},
												  centerOfRotation);

	m_kinematics.DesaturateWheelSpeeds(&states, Swordtip::Velocity::Maximums::Max_Speed);

	auto [rl, fl, fr, rr] = states;

	m_rearLeft.SetDesiredState(rl);
	m_frontLeft.SetDesiredState(fl);
	m_frontRight.SetDesiredState(fr);
	m_rearRight.SetDesiredState(rr);
}

void Drivetrain::AutoBalance(){
	angle_threshold = 0;

	current_angle = gyro->GetRoll();

	for (int i = 0; i < 5; ++i) {
		if (abs(current_angle) <= threshold_angles[i]) {
			angle_threshold += 1;
		}
	}

	if(current_angle > 0){
		chosen_speed = speed_multiplier[angle_threshold - 1];
	}else if(current_angle < 0){
		chosen_speed = -speed_multiplier[angle_threshold - 1];
	}

	Drive(0_fps,Swordtip::Velocity::Maximums::Max_Speed * chosen_speed,0_deg_per_s,0,Swordtip::Frame::RotationPoints::Center);
}

void Drivetrain::HaltRobot(){
	m_rearLeft.SetDesiredState({0_mps,0_deg});
	m_frontLeft.SetDesiredState({0_mps,0_deg});
	m_frontRight.SetDesiredState({0_mps,0_deg});
	m_rearRight.SetDesiredState({0_mps,0_deg});
}

void Drivetrain::SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode){
	m_rearLeft.SetNeutralMode(mode);
	m_frontLeft.SetNeutralMode(mode);
	m_frontRight.SetNeutralMode(mode);
	m_rearRight.SetNeutralMode(mode);
}

frc::Pose2d Drivetrain::UpdateOdometry() {return m_odometry.Update(gyro->GetRotation2d(), 
																{
																	m_rearLeft.GetPosition(), m_frontLeft.GetPosition(), 
																	m_frontRight.GetPosition(), m_rearRight.GetPosition()
																});
}

void Drivetrain::ResetOdometry() {
	m_odometry.ResetPosition(gyro->GetRotation2d(), 
							{
								m_rearLeft.GetPosition(), m_frontLeft.GetPosition(), 
								m_frontRight.GetPosition(), m_rearRight.GetPosition()
							},
							frc::Pose2d{0_m,0_m,0_deg});
}