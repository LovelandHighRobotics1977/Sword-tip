// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::Drive(DriveData data) {
	
	field_oriented_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(frc::ChassisSpeeds{data.forward, data.strafe, data.rotate}, gyro->GetRotation2d());
	robot_oriented_speeds = frc::ChassisSpeeds{data.forward, data.strafe, data.rotate};
	
	auto states = m_kinematics.ToSwerveModuleStates(data.fieldOriented ? field_oriented_speeds : robot_oriented_speeds);

	m_kinematics.DesaturateWheelSpeeds(&states, Swordtip::Velocity::Maximums::Max_Speed);

	auto [rl, fl, fr, rr] = states;

	m_rearLeft.SetDesiredState(rl);
	m_frontLeft.SetDesiredState(fl);
	m_frontRight.SetDesiredState(fr);
	m_rearRight.SetDesiredState(rr);
}

DriveData Drivetrain::AutoBalance(double angles[5], double speeds[5], double current_angle){
	angle_threshold = 0;

	for (int i = 0; i < 5; ++i) {
		if (abs(current_angle) <= angles[i]) {
			angle_threshold += 1;
		}
	}

	if(current_angle > 0){
		chosen_speed = speeds[angle_threshold];
	}else if(current_angle < 0){
		chosen_speed = -speeds[angle_threshold];
	}

	return(DriveData {0_fps,Swordtip::Velocity::Maximums::Max_Speed * chosen_speed,0_deg_per_s,0,Swordtip::Frame::RotationPoints::Center});
}

void Drivetrain::SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode){
	m_rearLeft.SetNeutralMode(mode);
	m_frontLeft.SetNeutralMode(mode);
	m_frontRight.SetNeutralMode(mode);
	m_rearRight.SetNeutralMode(mode);
}

frc::Pose2d Drivetrain::GetOdometry(){
	return m_odometry.GetPose();
}

frc::Pose2d Drivetrain::UpdateOdometry(){
	
	OdometryData data;

	data.angle = gyro->GetRotation2d();

	data.positions[0] = m_rearLeft.GetPosition();
	data.positions[1] = m_frontLeft.GetPosition();
	data.positions[2] = m_frontRight.GetPosition();
	data.positions[3] = m_rearRight.GetPosition();

	return m_odometry.Update(data.angle,data.positions);
}

void Drivetrain::ResetOdometry(frc::Pose2d position){

	OdometryData data;

	data.angle = gyro->GetRotation2d();

	data.positions[0] = m_rearLeft.GetPosition();
	data.positions[1] = m_frontLeft.GetPosition();
	data.positions[2] = m_frontRight.GetPosition();
	data.positions[3] = m_rearRight.GetPosition();

	m_odometry.ResetPosition(data.angle,data.positions,position);
}

