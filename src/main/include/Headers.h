#ifndef HEADERS_H
#define HEADERS_H

//c++
#include <iostream>
#include <math.h>
#include <cmath>

//frc kinematics
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

//units
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/dimensionless.h>

//motors and CAN devices
#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

//frc inputs
#include <frc/MathUtil.h> //for frc::ApplyDeadband
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/GenericHID.h>

//frc misc
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

//misc
#include <cameraserver/CameraServer.h>

//user defined
#include "Gyro.h"

namespace Swordtip{
	//robot misc specifications
	static constexpr auto Robot_Name = "   1977 : Swordtip";
	static constexpr double Drive_Gear_Ratio = 6.75; 											//	Gear ratio is L2 6.75:1
	static constexpr double Conversion_Factor = 4096.0/ 360.0;

	namespace Frame {
		static constexpr units::meter_t Length = 26_in;  										//  |Front left| frame to |rear left| frame
		static constexpr units::meter_t Width = 26_in;  										//  |Front left| frame to |front right| frame

		static constexpr units::meter_t Length_Offset = 2.625_in;  								//  distance from edge of frame to wheel
		static constexpr units::meter_t Width_Offset = 2.625_in;  								//  distance from edge of frame to wheel

		static constexpr units::meter_t Length_Location = ((Length/2)-Length_Offset);			//	distance from center to wheel |Left / Right|
		static constexpr units::meter_t Width_Location = ((Width/2)-Width_Offset);				//	distance from center to wheel |Front / Back|

		static constexpr frc::Translation2d Center = {0_in,0_in};   							//  position of the center of the robot
		static constexpr frc::Translation2d Tower = {-10.5_in,0_in};							//  position of the robot tower
	}
	



	namespace Velocity {
		namespace Maximums {
			static constexpr units::feet_per_second_t Max_Speed = 3_fps;                 	//  max horizontal velocity 16.3 feet per second
			static constexpr units::degrees_per_second_t Max_Rotation = 360_deg_per_s;   	//  max rotational velocity ~763 degrees per second
		}
		
		namespace Rotation {
			static constexpr units::degrees_per_second_t Slow =  120_deg_per_s;      		//  250 degrees per second
			static constexpr units::degrees_per_second_t Medium = 180_deg_per_s;    		//  380 degrees per second
			static constexpr units::degrees_per_second_t Fast = 360_deg_per_s;  			//  760 degrees per second
		}
	}




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
};

#endif /* !HEADERS_H */
