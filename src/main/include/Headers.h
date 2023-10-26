#ifndef HEADERS_H
#define HEADERS_H

// c++
#include <iostream>
#include <math.h>
#include <cmath>
#include <variant>
#include <vector>
#include <functional>

// frc kinematics
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

// units
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include <units/math.h>

// motors and CAN devices
#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

// frc inputs
#include <frc/MathUtil.h> // for frc::ApplyDeadband
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/GenericHID.h>

// frc misc
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

// cameras
#include <cameraserver/CameraServer.h>

// 1977 defined libraries
#include "Gyro.h"

// Variables for the robot swordtip
namespace Swordtip{
	/**
	 * Misc robot variables
	*/
	namespace Misc {
		static constexpr auto Robot_Name = "   1977 : Swordtip";
		static constexpr double Conversion_Factor = 4096.0/ 360.0;
	}
	
	/**
	 * Frame measurments and important physical locations
	*/
	namespace Frame {
		namespace Measurments {
			static constexpr units::inch_t Length = 26_in;  									//  |Front left| frame to |rear left| frame
			static constexpr units::inch_t Width = 26_in;  										//  |Front left| frame to |front right| frame

			static constexpr units::inch_t Length_Offset = 2.625_in;  							//  distance from edge of frame to wheel
			static constexpr units::inch_t Width_Offset = 2.625_in;  							//  distance from edge of frame to wheel

			static constexpr units::inch_t Length_Location = ((Length/2)-Length_Offset);		//	distance from center to wheel |Left / Right|
			static constexpr units::inch_t Width_Location = ((Width/2)-Width_Offset);			//	distance from center to wheel |Front / Back|
			
			static const units::foot_t Turning_Circle = units::foot_t{((2 * M_PI) * std::sqrt((std::pow(2 * Frame::Measurments::Length_Location.value(), 2) + std::pow(2 * Frame::Measurments::Width_Location.value(), 2))))/12};
		}
		namespace RotationPoints {
			static constexpr frc::Translation2d Center = {0_in,0_in};   						//  position of the center of the robot
			static constexpr frc::Translation2d Tower = {-10.5_in,0_in};						//  position of the robot tower
		}	
		namespace ModuleStats {
			static constexpr double Drive_RPMs = 6380;
			static constexpr units::inch_t Wheel_Radius = 2_in;
			static constexpr double Drive_Gear_Ratio = 6.75;
		}				
	}
	/**
	 * Velocity maximums and presets
	*/
	namespace Velocity {
		/**
		 * Velocity Maximums
		*/
		namespace Maximums {
			// Max horizontal velocity of ~16 feet per second
			static constexpr auto True_Max_Speed = units::feet_per_second_t{(((2 * Frame::ModuleStats::Wheel_Radius * M_PI) * (Frame::ModuleStats::Drive_RPMs / Frame::ModuleStats::Drive_Gear_Ratio))/ 60 / 12).value()};
			// Max rotational velocity of ~773 degrees per second
			static const auto True_Max_Rotation = units::degrees_per_second_t{(360 * ((True_Max_Speed) / (Frame::Measurments::Turning_Circle))).value()};
		}
		/**
		 * Rotational velocity presets
		*/
		namespace Rotation {
			static const units::degrees_per_second_t None = 0_deg_per_s;  						//  0 degrees per second
			static const units::degrees_per_second_t Slow =  Maximums::True_Max_Rotation / 3;   //  260 degrees per second
			static const units::degrees_per_second_t Medium = Maximums::True_Max_Rotation / 2;  //  370 degrees per second
			static const units::degrees_per_second_t Fast = Maximums::True_Max_Rotation;  		//  770 degrees per second
		}
	}

	/**
	 * PIDF values for the drivetrain falcon 500s
	*/
	namespace PIDF {
		namespace Drive {
				static constexpr double P = 0.001;
				static constexpr double I = 0;
				static constexpr double D = 0.005;
				static constexpr double F = 1;
		}
		namespace Angle {
				static constexpr double P = 1.7;
				static constexpr double I = 0.0016;
				static constexpr double D = 160;
				static constexpr double F = 0;
		}
	}

	/**
	 * Autonomus variables
	*/
	namespace Autonomous {
		namespace Variable {
			static constexpr double timer_resolution = 0.1;
		}
	}
}

// Argument abstraction data types
/**
 * Data for driving the robot
 * @param forward Forward movement of the robot in meters/sec.
 * @param strafe Sideways movement of the robot in meters/sec.
 * @param rotate Rotational movement of the robot in degrees/sec
 * @param fieldRelative Is the robot being driven field oriented?
 * @param centerOfRotation Robot center of rotation.
*/
struct DriveData{
	units::feet_per_second_t forward = 0_fps;
	units::feet_per_second_t strafe = 0_fps;
	units::angular_velocity::degrees_per_second_t rotate = 0_deg_per_s;
	bool field_oriented = true;
	frc::Translation2d centerOfRotation = Swordtip::Frame::RotationPoints::Center;
};

/**
 * Data for updating and resetting odometry
 * @param angle
 * @param rearLeft_Position 
 * @param frontLeft_Position 
 * @param frontRight_Position 
 * @param rearRight_Position 
*/
struct OdometryData{
	frc::Rotation2d angle;
	std::array<frc::SwerveModulePosition,4> positions;
};

/**
 * Data for setting the arm speed
 * @param LOW Aim low?
 * @param MID Aim mid?
 * @param HIGH Aim high?
*/
struct ArmData {
	bool Target[3] = {false,false,false};
};

/**
 * Data for an autonomous task
 * @param start_function the function to execute at the start time
 * @param end_function the function to execute at the end time
 * @param start_time start time, defaults to 0 seconds
 * @param end_time end time, defaults to 15 seconds
*/
struct Task{
	const std::function<void()>& start_function;
	const std::function<void()>& end_function;
	double start_time = 0;
	double end_time = 15;
};


// Variables to make shit look nice

static constexpr int UNSET = 0; // Not on a team :(
static constexpr int GREEN = 0; // OH FUCK
static constexpr int RED = 1; // RED team!
static constexpr int BLUE = 2; // BLUE team!

static constexpr int SIT = 0; // Sit still and do nothing
static constexpr int SHORT = 1; // Take a wide radius around the charge station and skedaddle
static constexpr int MIDDLE = 2; // Go over the charge station at a slow speed
static constexpr int LONG = 3; // Take a small distance to squeeze by the charge station and skedaddle.  Closest to opponent human player

static constexpr ArmData NONE = {false,false,false}; // Aim nowhere and do not shoot :(
static constexpr ArmData LOW = {1, false, false}; // Aim for a low node (WILL hit low 100% of the time)
static constexpr ArmData MID = {false, 1, false}; // Aim for a mid node (Finicky, 20% success rate)
static constexpr ArmData HIGH = {false, false, 1}; // Aim for high node (Consistent, 90% success rate)

/**
 * Autonomous selector data
 * @param node Node to aim at (LOW, MID, HIGH)
 * @param path Auto to execute (SHORT, MID, LONG)
 * @param alliance Our alliance color (RED, BLUE)
*/
struct SelectedAuto {
	/**
		 * @param NONE Do not shoot :(
		 * @param LOW Aim for a low node (WILL hit low 100% of the time)
		 * @param MID Aim for a mid node (Finicky, 20% success rate)
		 * @param HIGH Aim for high node (Consistent, 90% success rate)
	*/
	ArmData node = NONE;

	/**
		 * @param SIT Sit still and do absolutely nothing for 15 seconds :(
		 * @param SHORT Take a wide radius around the charge station and skedaddle
		 * @param MIDDLE Go over the charge station at a slow speed
		 * @param LONG Take a small distance to squeeze by the charge station and skedaddle.  Closest to opponent human player
	*/
	int path = SIT;

	/**
		 * @param UNSET No team :(
		 * @param GREEN OH FUCK
		 * @param RED RED TEAM !!!
		 * @param BLUE BLOOOOO TEAM !!!
	*/
	int alliance = UNSET;
};

#endif /* !HEADERS_H */