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
//#include <cameraserver/CameraServer.h>

//user defined
#include "Gyro.h"

//global constants
static const double conversionFactor = 4096.0/ 360.0;
static constexpr double kDriveGearRatio = 6.75; //Gear ratio is L2 6.75:1
static constexpr units::meter_t kRobotWidth = 2_ft;  //  |Front left| wheel to |front right| wheel
static constexpr units::meter_t kRobotLength = 2_ft;  //  |Front left| wheel to |rear left| wheel
static constexpr frc::Translation2d kCenterOfRobot = {0_in,0_in};           //  position of the center of the robot
static constexpr frc::Translation2d kCenterOfMass = {6.5_in,0_in};          //  position of the center of mass (in this case the robot tower)
static constexpr units::meters_per_second_t kMaxSpeed = 1.0_mps;             //  1 "meter" per second
static constexpr units::degrees_per_second_t kSlowRotation = 45_deg_per_s;   //  45 "degrees" per second
static constexpr units::degrees_per_second_t kMediumRotation = 60_deg_per_s; //  60 "degrees" per second
static constexpr units::degrees_per_second_t kFastRotation = 180_deg_per_s;  //  180 "degrees" per second

#endif /* !HEADERS_H */