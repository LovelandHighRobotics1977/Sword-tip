// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 * 
 */
class Drivetrain {
	public:
		Drivetrain(){
			gyro->Reset();
		}
		
		/**
		 * Drives the swerve robot
		 * @param forward Forward movement of the robot in meters/sec.
		 * @param strafe Sideways movement of the robot in meters/sec.
		 * @param rotate Rotational movement of the robot in degrees/sec
		 * @param fieldRelative Is the robot being driven field oriented?
		 * @param centerOfRotation Robot center of rotation.
		*/
		void Drive(units::meters_per_second_t forward, units::meters_per_second_t strafe, units::degrees_per_second_t rotate, bool fieldRelative, frc::Translation2d centerOfRotation);
		/**
		 * Updates the swerve drive odometry
		 * @param robotAngle the angle of the robot as a rotation2D
		 * @return returns a pose2d of the robot's position on the field.
		*/

		void AutoBalance();

		void HaltRobot();

		void SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode);

		frc::Pose2d UpdateOdometry();

		void ResetOdometry();
		

	private:
		Gyro* gyro = Gyro::GetInstance();

		double current_angle;

		double threshold_angles[5] = {  30,   20,   10,    5,    2};  // Threshold angles in degrees
		double speed_multiplier[5] = {0.30, 0.15, 0.10, 0.08, 0.00};  // Associated motor speeds

		int angle_threshold;

		double chosen_speed;

		//gear ratio is L2 6.75:1

		// This is not how it should be but doing it "correctly" (-+,++,+-,--) causes
		// the wheels to form an "X" instead of diamond while turning.
		// The x coordinate is the inverse of the correct X coordinate
		// It's wrong but (++,-+,--,+-) it works, no touchy.
		
		SwerveModule m_frontRight{0, 1, 2, -112.588};
		frc::Translation2d m_frontRightLocation{-Swordtip::Frame::Measurments::Length_Location, -Swordtip::Frame::Measurments::Width_Location};

		SwerveModule m_rearRight{3, 4, 5, 101.777};
		frc::Translation2d m_rearRightLocation{+Swordtip::Frame::Measurments::Length_Location, -Swordtip::Frame::Measurments::Width_Location};

		SwerveModule m_rearLeft{6, 7, 8, 111.0059};
		frc::Translation2d m_rearLeftLocation{+Swordtip::Frame::Measurments::Length_Location, +Swordtip::Frame::Measurments::Width_Location};
		
		SwerveModule m_frontLeft{9, 10, 11, 149.6777};
		frc::Translation2d m_frontLeftLocation{-Swordtip::Frame::Measurments::Length_Location, +Swordtip::Frame::Measurments::Width_Location};

		frc::SwerveDriveKinematics<4> m_kinematics{m_rearLeftLocation, m_frontLeftLocation, m_frontRightLocation, m_rearRightLocation};

		frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, 
												gyro->GetRotation2d(),
												{
											   		m_rearLeft.GetPosition(),  m_frontLeft.GetPosition(), 
													m_frontRight.GetPosition(), m_rearRight.GetPosition()
												},
												frc::Pose2d{0_m,0_m,0_deg}
											};
};