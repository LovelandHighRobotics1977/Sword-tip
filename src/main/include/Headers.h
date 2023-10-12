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

// motors and CAN devices
#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

// frc inputs
#include <frc/MathUtil.h> // for frc::ApplyDeadband
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/GenericHID.h>

// frc misc
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

// misc
#include <cameraserver/CameraServer.h>

// user defined
#include "Gyro.h"

// Variables for the robot swordtip
namespace Swordtip{
	/**
	 * Misc robot variables
	*/
	namespace Misc {
		static constexpr auto Robot_Name = "   1977 : Swordtip";
		static constexpr double Drive_Gear_Ratio = 6.75; 										//	Gear ratio is L2 6.75:1
		static constexpr double Conversion_Factor = 4096.0/ 360.0;
	}
	
	/**
	 * Frame measurments and important physical locations
	*/
	namespace Frame {
		namespace Measurments {
			static constexpr units::meter_t Length = 26_in;  									//  |Front left| frame to |rear left| frame
			static constexpr units::meter_t Width = 26_in;  									//  |Front left| frame to |front right| frame

			static constexpr units::meter_t Length_Offset = 2.625_in;  							//  distance from edge of frame to wheel
			static constexpr units::meter_t Width_Offset = 2.625_in;  							//  distance from edge of frame to wheel

			static constexpr units::meter_t Length_Location = ((Length/2)-Length_Offset);		//	distance from center to wheel |Left / Right|
			static constexpr units::meter_t Width_Location = ((Width/2)-Width_Offset);			//	distance from center to wheel |Front / Back|
		}
		namespace RotationPoints {
			static constexpr frc::Translation2d Center = {0_in,0_in};   						//  position of the center of the robot
			static constexpr frc::Translation2d Tower = {-10.5_in,0_in};						//  position of the robot tower
		}					
	}
	/**
	 * Velocity maximums and presets
	*/
	namespace Velocity {
		namespace Maximums {
			static constexpr units::feet_per_second_t Max_Speed = 11_fps;                		//  max horizontal velocity 16.3 feet per second
			static constexpr units::degrees_per_second_t Max_Rotation = 420_deg_per_s;   		//  max rotational velocity ~763 degrees per second
		}
		/**
		 * Rotational velocity presets
		*/
		namespace Rotation {
			static constexpr units::degrees_per_second_t Slow =  Maximums::Max_Rotation / 3;    //  140 degrees per second
			static constexpr units::degrees_per_second_t Medium = Maximums::Max_Rotation / 2;   //  210 degrees per second
			static constexpr units::degrees_per_second_t Fast = Maximums::Max_Rotation;  		//  420 degrees per second
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
		namespace Tasks {

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
	bool fieldOriented = 0;
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

union Argument{
	OdometryData odometry;
	DriveData drive;
};

#endif /* !HEADERS_H */