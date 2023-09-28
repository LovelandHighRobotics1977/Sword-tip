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

	rl = (frc::SwerveModuleState{rl.speed, frc::Rotation2d{units::angle::degree_t{fmod((rl.angle.Degrees().value()+360),360)}}});
	fl = (frc::SwerveModuleState{fl.speed, frc::Rotation2d{units::angle::degree_t{fmod((fl.angle.Degrees().value()+360),360)}}});
	fr = (frc::SwerveModuleState{fr.speed, frc::Rotation2d{units::angle::degree_t{fmod((fr.angle.Degrees().value()+360),360)}}});
	rr = (frc::SwerveModuleState{rr.speed, frc::Rotation2d{units::angle::degree_t{fmod((rr.angle.Degrees().value()+360),360)}}});

	m_rearLeft.SetDesiredState(rl);
	m_frontLeft.SetDesiredState(fl);
	m_frontRight.SetDesiredState(fr);
	m_rearRight.SetDesiredState(rr);
}

void Drivetrain::AutoBalance(){
	current_angle_raw = gyro->GetRoll();

	current_angle = abs(current_angle_raw);

	leaning_left = current_angle_raw > 0;
	leaning_right = current_angle_raw < 0;

	angle_threshold = (((current_angle <= threshold_angles[0])+((current_angle <= threshold_angles[1]))+((current_angle <= threshold_angles[2]))+((current_angle <= threshold_angles[3]))+((current_angle <= threshold_angles[4]))));

	if(leaning_left){
		Drive(0_mps,1_mps * speed_multiplier[angle_threshold - 1],0_deg_per_s,0,Swordtip::Frame::Center);
	}else if(leaning_right){
		Drive(0_mps,-1_mps * speed_multiplier[angle_threshold - 1],0_deg_per_s,0,Swordtip::Frame::Center);
	}
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