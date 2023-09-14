// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

SwerveModule::SwerveModule(const int driveMotorID,     const int angleMotorID,       const int angleEncoderID, double magnetOffset)
					  : m_driveMotor{driveMotorID}, m_angleMotor{angleMotorID}, m_angleEncoder{angleEncoderID} {

	//--------

	m_driveMotor.SetNeutralMode(NeutralMode::Brake);

	m_driveMotor.ConfigFactoryDefault();
	m_driveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
	m_driveMotor.Config_kP(0, drivekP);
	m_driveMotor.Config_kI(0, drivekI);
	m_driveMotor.Config_kD(0, drivekD);
	m_driveMotor.Config_kF(0, drivekF);
	m_driveMotor.ConfigNominalOutputForward(0);
	m_driveMotor.ConfigNominalOutputReverse(0);
	m_driveMotor.ConfigPeakOutputForward(1);
	m_driveMotor.ConfigPeakOutputReverse(-1);

	//--------

	m_angleMotor.SetSensorPhase(true);
	m_angleMotor.SetNeutralMode(NeutralMode::Brake);

	m_angleMotor.ConfigFactoryDefault();
	m_angleMotor.ConfigRemoteFeedbackFilter(angleEncoderID, RemoteSensorSource(13), 0, 0);
	m_angleMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0
	m_angleMotor.Config_kP(0, anglekP);
	m_angleMotor.Config_kI(0, anglekI);
	m_angleMotor.Config_kD(0, anglekD);
	m_angleMotor.Config_kF(0, anglekF);
	m_angleMotor.Config_IntegralZone(0, 20);
	m_angleMotor.ConfigNominalOutputForward(0);
	m_angleMotor.ConfigNominalOutputReverse(0);
	m_angleMotor.ConfigPeakOutputForward(1);
	m_angleMotor.ConfigPeakOutputReverse(-1);

	//--------

	m_angleEncoder.ConfigMagnetOffset(magnetOffset);
	
	CANID = angleEncoderID;

	m_angleEncoder.SetPositionToAbsolute();
	m_angleMotor.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);

	m_angleMotor.Set(TalonFXControlMode::Position,0);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
	return {units::meter_t{(m_driveMotor.GetSelectedSensorPosition())*(((4*M_PI)/kDriveGearRatio)/2048)}, gyro->GetRotation2d()};
}

frc::Rotation2d SwerveModule::getAngle() {
	return frc::Rotation2d{units::degree_t{-m_angleEncoder.GetPosition()}};
}

frc::SwerveModuleState SwerveModule::Optimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle, bool optimize){
	
	frc::SwerveModuleState wpiOptimizedState = frc::SwerveModuleState::Optimize(desiredState, SwerveModule::getAngle());

	if(optimize){
		return frc::SwerveModuleState{wpiOptimizedState.speed,frc::Rotation2d{units::angle::degree_t{fmod((wpiOptimizedState.angle.Degrees().value()+360),360)}}};
	}else{
		return frc::SwerveModuleState{desiredState.speed,frc::Rotation2d{units::angle::degree_t{fmod((desiredState.angle.Degrees().value()+360),360)}}};
	}
}

void SwerveModule::SetDesiredState(
	const frc::SwerveModuleState& desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		auto const [optimized_speed, optimized_angle] = Optimize(desiredState, getAngle(), true);

		switch (CANID){
		case 2:
			frc::SmartDashboard::PutNumber("Rear Left angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Rear Left speed", optimized_speed.value());
			break;
		case 5:
			frc::SmartDashboard::PutNumber("Front Left angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Front Left speed", optimized_speed.value());
			break;
		case 8:
			frc::SmartDashboard::PutNumber("Front Right angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Front Right speed", optimized_speed.value());
			break;
		case 11:
			frc::SmartDashboard::PutNumber("Rear Right angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Rear Right speed", optimized_speed.value());
			break;
		}

		// Set the motor outputs.
		m_driveMotor.Set((double) optimized_speed);
		m_angleMotor.Set(TalonFXControlMode::Position, optimized_angle.Degrees().value()*(conversionFactor));
}