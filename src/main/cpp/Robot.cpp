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
	m_cubeArm.SetSpeed(0,0,0);

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
	m_cubeArm.SetSpeed(0,1,0);

	// Prepare timer and gyro for auto
	timer.Restart();
	gyro->Reset();

	// Select which autonomous to do
	auto_task = 1;
	auto_alliance = RED;

}
void Robot::AutonomousPeriodic() {

	#pragma region COMMON
	Task fire_cube = {
		[this](){ m_cubeArm.SetIntake(0,1); }, 
		[this](){ m_cubeArm.SetIntake(0,0); },
		0, 1
	};
	#pragma endregion COMMON

	#pragma region DO_NOTHING
	Task do_nothing = {
		[this](){ pass(); }, 
		[this](){ pass(); },
		0,15
	};
	Task do_nothing_auto[1] = {
		do_nothing
	};
	#pragma endregion DO_NOTHING

	#pragma region RED

	#pragma region RED Slot ONE
	Task s1_RED_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,1_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task s1_RED_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task s1_RED_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task s1_RED_auto[4] = {
		fire_cube,
		s1_RED_leave_zone,
		s1_RED_move_forward,
		s1_RED_wait_for_teleop
	};
	#pragma endregion RED Slot ONE

	#pragma region RED Slot TWO
	Task s2_RED_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {0.9_fps}); }, 
		[this](){ pass(); },
		1, 6
	};
	Task s2_RED_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		6
	};
	Task s2_RED_auto[3] = {
		fire_cube,
		s2_RED_leave_zone,
		s2_RED_wait_for_teleop
	};
	#pragma endregion RED Slot TWO

	#pragma region RED Slot THREE
	Task s3_RED_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,-0.5_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task s3_RED_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task s3_RED_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task s3_RED_auto[4] = {
		fire_cube,
		s3_RED_leave_zone,
		s3_RED_move_forward,
		s3_RED_wait_for_teleop
	};
	#pragma endregion RED Slot THREE

	#pragma endregion RED

	#pragma region BLUE

	#pragma region BLUE Slot ONE
	Task s1_BLUE_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,-1_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task s1_BLUE_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task s1_BLUE_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task s1_BLUE_auto[4] = {
		fire_cube,
		s1_BLUE_leave_zone,
		s1_BLUE_move_forward,
		s1_BLUE_wait_for_teleop
	};
	#pragma endregion BLUE Slot ONE

	#pragma region BLUE Slot TWO
	Task s2_BLUE_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {0.9_fps}); }, 
		[this](){ pass(); },
		1, 6
	};
	Task s2_BLUE_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		6
	};
	Task s2_BLUE_auto[3] = {
		fire_cube,
		s2_BLUE_leave_zone,
		s2_BLUE_wait_for_teleop
	};
	#pragma endregion BLUE Slot TWO

	#pragma region BLUE Slot THREE
	Task s3_BLUE_leave_zone = {
		[this](){ m_swerve.Drive(DriveData {1_fps,0.5_fps}); }, 
		[this](){ pass(); },
		1, 2
	};
	Task s3_BLUE_move_forward = {
		[this](){ m_swerve.Drive(DriveData {3_fps}); },
		[this](){ pass(); },
		2, 3
	};
	Task s3_BLUE_wait_for_teleop = {
		[this](){ m_swerve.Drive(DriveData {}); },
		[this](){ pass(); },
		3
	};
	Task s3_BLUE_auto[4] = {
		fire_cube,
		s3_BLUE_leave_zone,
		s3_BLUE_move_forward,
		s3_BLUE_wait_for_teleop
	};
	#pragma endregion BLUE Slot THREE

	#pragma endregion BLUE

	#pragma region EXECUTE
	switch (auto_alliance){
		// RED TEAM
		case 1:
			switch (auto_task){
				case 1:
					for (int i = 0; i < (int) std::size(s1_RED_auto); i++){
						ExecuteTask(timer.Get().value(),s1_RED_auto[i]);
					}
					break;
				case 2:
					for (int i = 0; i < (int) std::size(s2_RED_auto); i++){
						ExecuteTask(timer.Get().value(),s2_RED_auto[i]);
					}
					break;
				case 3:
					for (int i = 0; i < (int) std::size(s3_RED_auto); i++){
						ExecuteTask(timer.Get().value(),s3_RED_auto[i]);
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
			switch (auto_task){
				case 1:
					for (int i = 0; i < (int) std::size(s1_BLUE_auto); i++){
						ExecuteTask(timer.Get().value(),s1_BLUE_auto[i]);
					}
					break;
				case 2:
					for (int i = 0; i < (int) std::size(s2_BLUE_auto); i++){
						ExecuteTask(timer.Get().value(),s2_BLUE_auto[i]);
					}
					break;
				case 3:
					for (int i = 0; i < (int) std::size(s3_BLUE_auto); i++){
						ExecuteTask(timer.Get().value(),s3_BLUE_auto[i]);
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
	m_cubeArm.SetSpeed(shooter.speed_slow, shooter.speed_medium, shooter.speed_fast);
	
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