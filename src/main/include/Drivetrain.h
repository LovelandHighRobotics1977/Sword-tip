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
		 * Drives the swerve robot by taking in a DriveData array
		 * @param DriveData A DriveData array used to drive the robot
		*/
		void Drive(DriveData);

		/**
		 * Calculates the DriveData to auto balance the robot on the charge station
		 * @param angles Angles to change speed on
		 * @param speeds Speed to set in relation to current angle
		 * @return Returns a DriveData array to drive the robot
		*/
		DriveData AutoBalance(double angles[5], double speeds[5], double current_angle);

		/**
		 * Sets the neutral mode of the drivetrain
		 * @param mode Phoenix library motor speed to set motors to
		*/
		void SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode);

		/**
		 * Gets the swerve drive odometry values
		 * @return returns a pose2d of the robot's position on the field.
		*/
		frc::Pose2d GetOdometry();

		/**
		 * Updates the swerve drive odometry and returns the value
		 * @return returns a pose2d of the robot's position on the field.
		*/
		frc::Pose2d UpdateOdometry();

		/**
		 * Resets the odometry
		 * @param position The pose2D of the position of the robot
		*/
		void ResetOdometry(frc::Pose2d position);

	private:

		Gyro* gyro = Gyro::GetInstance();

		int angle_threshold;

		double chosen_speed;

		frc::ChassisSpeeds field_oriented_speeds;
		frc::ChassisSpeeds robot_oriented_speeds;

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

		// Odometry Ballshart

		frc::SwerveDriveOdometry<4> m_odometry{m_kinematics,
												gyro->GetRotation2d(),
												{
											   		m_rearLeft.GetPosition(),  m_frontLeft.GetPosition(), 
													m_frontRight.GetPosition(), m_rearRight.GetPosition()
												},
												frc::Pose2d{0_m,0_m,0_deg}
											};
};