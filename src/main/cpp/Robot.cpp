// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void pass(){}

void Robot::RobotInit() {

	frc::SmartDashboard::PutString("Match State","   Disabled");
	frc::SmartDashboard::PutString("Robot Name",Swordtip::Misc::Robot_Name);

	frc::SmartDashboard::PutNumber("X_POS", 0);
	frc::SmartDashboard::PutNumber("Y_POS", 0);

	frc::SmartDashboard::PutNumber("X_VEL", 0);
	frc::SmartDashboard::PutNumber("Y_VEL", 0);

	m_swerve.Drive(DriveData {});
	m_cubeArm.SetIntake(0,0);
	m_cubeArm.SetAngle(0,0);
	m_cubeArm.SetSpeed(ArmData {});

	timer.Start();

	/*
	r_driveCam = frc::CameraServer::StartAutomaticCapture(0);
	r_armCam = frc::CameraServer::StartAutomaticCapture(1);

	r_driveCam.SetPixelFormat(cs::VideoMode::PixelFormat::kYUYV);
	r_armCam.SetPixelFormat(cs::VideoMode::PixelFormat::kYUYV);
	*/
}
void Robot::RobotPeriodic() {

	frc::SmartDashboard::PutNumber("Angle", static_cast<int>(gyro->GetYaw()));

	robot_position = m_swerve.UpdateOdometry();

}

void Robot::ExecuteTask(double current_time, Task task){
	if((current_time > task.start_time) && (current_time < task.start_time + Swordtip::Autonomous::Variable::timer_resolution)){
		task.start_function();
	}

	if((current_time < task.end_time) && (current_time > task.end_time - Swordtip::Autonomous::Variable::timer_resolution)){
		task.end_function();
	}
}

void Robot::AutonomousInit() {
	frc::SmartDashboard::PutString("Match State","   Autonomous");

	// Autonomous mode specific configs
	m_swerve.SetNeutralMode(Brake);
	m_swerve.Drive(DriveData {});
	m_swerve.ResetOdometry({0_m,0_m,0_deg});

	// Prepare timer and gyro for auto
	timer.Restart();
	gyro->Reset();

}
void Robot::AutonomousPeriodic() {

	#pragma region COMMON
	Task set_arm = {
		[this](){ m_cubeArm.SetSpeed(Auto.node); }, 
		[this](){ pass(); },
		0, 1
	};
	Task fire_cube = {
		[this](){ m_cubeArm.SetIntake(0,1); }, 
		[this](){ m_cubeArm.SetIntake(0,0); },
		0.1, 1
	};
	#pragma endregion COMMON

	#pragma region NOTHING
	Task do_nothing = {
		[this](){ pass(); }, 
		[this](){ pass(); },
		0,15
	};
	Task do_nothing_auto[1] = {
		do_nothing
	};
	#pragma endregion NOTHING

	#pragma region RED

	#pragma region RED SHORT
	Task short_RED_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,-0.5_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task short_RED_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task short_RED_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task short_RED_auto[5] = {
		set_arm,
		fire_cube,
		short_RED_leave_zone,
		short_RED_move_forward,
		short_RED_wait_for_teleop
	};
	#pragma endregion RED SHORT

	#pragma region RED MIDDLE
	Task middle_RED_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {0.9_fps}); }, 
		[this](){ pass(); },
		1, 6
	};
	Task middle_RED_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		6
	};
	Task middle_RED_auto[4] = {
		set_arm,
		fire_cube,
		middle_RED_leave_zone,
		middle_RED_wait_for_teleop
	};
	#pragma endregion RED MIDDLE

	#pragma region RED LONG
	Task long_RED_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,0.2_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task long_RED_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task long_RED_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task long_RED_auto[5] = {
		set_arm,
		fire_cube,
		long_RED_leave_zone,
		long_RED_move_forward,
		long_RED_wait_for_teleop
	};
	#pragma endregion RED LONG

	#pragma endregion RED

	#pragma region BLUE

	#pragma region BLUE SHORT
	Task short_BLUE_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,0.5_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task short_BLUE_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task short_BLUE_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task short_BLUE_auto[5] = {
		set_arm,
		fire_cube,
		short_BLUE_leave_zone,
		short_BLUE_move_forward,
		short_BLUE_wait_for_teleop
	};
	#pragma endregion BLUE SHORT

	#pragma region BLUE MIDDLE
	Task middle_BLUE_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {0.9_fps}); }, 
		[this](){ pass(); },
		1, 6
	};
	Task middle_BLUE_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		6
	};
	Task middle_BLUE_auto[4] = {
		set_arm,
		fire_cube,
		middle_BLUE_leave_zone,
		middle_BLUE_wait_for_teleop
	};
	#pragma endregion BLUE MIDDLE

	#pragma region BLUE LONG
	Task long_BLUE_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,-0.2_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task long_BLUE_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task long_BLUE_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task long_BLUE_auto[5] = {
		set_arm,
		fire_cube,
		long_BLUE_leave_zone,
		long_BLUE_move_forward,
		long_BLUE_wait_for_teleop
	};
	#pragma endregion BLUE LONG

	#pragma endregion BLUE

	#pragma region EXECUTE
	switch (Auto.alliance){
		// RED TEAM
		case 1:
			switch (Auto.path){
				case 1:
					for (int i = 0; i < (int) std::size(short_RED_auto); i++){
						ExecuteTask(timer.Get().value(),short_RED_auto[i]);
					}
					break;
				case 2:
					for (int i = 0; i < (int) std::size(middle_RED_auto); i++){
						ExecuteTask(timer.Get().value(),middle_RED_auto[i]);
					}
					break;
				case 3:
					for (int i = 0; i < (int) std::size(long_RED_auto); i++){
						ExecuteTask(timer.Get().value(),long_RED_auto[i]);
					}
					break;
				default:
					for (int i = 0; i < (int) std::size(do_nothing_auto); i++){
						ExecuteTask(timer.Get().value(),do_nothing_auto[i]);
					}
					break;
			}
			break;

		// BLUE TEAM
		
		case 2:
			switch (Auto.path){
				case 1:
					for (int i = 0; i < (int) std::size(short_BLUE_auto); i++){
						ExecuteTask(timer.Get().value(),short_BLUE_auto[i]);
					}
					break;
				case 2:
					for (int i = 0; i < (int) std::size(middle_BLUE_auto); i++){
						ExecuteTask(timer.Get().value(),middle_BLUE_auto[i]);
					}
					break;
				case 3:
					for (int i = 0; i < (int) std::size(long_BLUE_auto); i++){
						ExecuteTask(timer.Get().value(),long_BLUE_auto[i]);
					}
					break;
				default:
					for (int i = 0; i < (int) std::size(do_nothing_auto); i++){
						ExecuteTask(timer.Get().value(),do_nothing_auto[i]);
					}
					break;
			}
			break;

		default:
			for (int i = 0; i < (int) std::size(do_nothing_auto); i++){
				ExecuteTask(timer.Get().value(),do_nothing_auto[i]);
			}
			break;

	}
	#pragma endregion EXECUTE

}

void Robot::TeleopInit() {

	frc::SmartDashboard::PutString("Match State","   TeleOperated");

	m_swerve.SetNeutralMode(Brake);

}
void Robot::TeleopPeriodic() {
	// Update controller inputs
	driver.update();
	shooter.update();

	// Reset Gyro
	if(driver.gyro_reset){
		gyro->Reset();
	}

	// Control the mechanism
	m_cubeArm.SetAngle(shooter.angle_up, shooter.angle_down);
	m_cubeArm.SetIntake(shooter.intake_in, shooter.intake_out);
	m_cubeArm.SetSpeed({shooter.speed_slow, shooter.speed_medium, shooter.speed_fast});
	
	// Select rotation speed from triggers
	switch ((driver.trigger_one + driver.trigger_two)){
	case 0:
		center_of_rotation = Swordtip::Frame::RotationPoints::Center;
		rotation_speed = Swordtip::Velocity::Rotation::Slow;
		break;
	
	case 1:
		center_of_rotation = Swordtip::Frame::RotationPoints::Center;
		rotation_speed = Swordtip::Velocity::Rotation::Medium;
		break;

	case 2:
		center_of_rotation = Swordtip::Frame::RotationPoints::Center;
		rotation_speed = Swordtip::Velocity::Rotation::Fast;
		break;
	}

	// Apply deadbands and slew rate limiters
	forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(driver.forward, 0.2)) * driver.throttle) * Swordtip::Velocity::Maximums::Max_Speed;
	strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(driver.strafe, 0.2)) * driver.throttle) * Swordtip::Velocity::Maximums::Max_Speed;
	rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(driver.rotate, 0.3)) * sqrt(driver.throttle)) * rotation_speed;

	// Drive the swerve modules
	if(driver.emergency_stop){
		// Emergency braking (Button 5)
		m_swerve.Drive(DriveData {});
	}else{
		// Normal Drive
		m_swerve.Drive({forward, strafe, rotate, driver.field_oriented, center_of_rotation});
	}

	// Put values to SmartDashboard
	frc::SmartDashboard::PutNumber("throttle", driver.throttle);
	
	frc::SmartDashboard::PutNumber("X_POS", robot_position.X().value());
	frc::SmartDashboard::PutNumber("Y_POS", robot_position.Y().value());

	frc::SmartDashboard::PutNumber("R_VEL", (double) rotate);
	frc::SmartDashboard::PutNumber("X_VEL", gyro->GetVelocityX());
	frc::SmartDashboard::PutNumber("Y_VEL", gyro->GetVelocityY());
}

void Robot::DisabledInit() {
	frc::SmartDashboard::PutString("Match State","   Disabled");
	m_swerve.Drive(DriveData {});
}
void Robot::DisabledPeriodic() {

}

void Robot::TestInit() {
	frc::SmartDashboard::PutString("Match State","   Test");
}
void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRc_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif