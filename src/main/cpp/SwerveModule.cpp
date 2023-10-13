// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

SwerveModule::SwerveModule(const int driveMotorID,     const int angleMotorID,       const int angleEncoderID, double magnetOffset)
					  : m_driveMotor{driveMotorID}, m_angleMotor{angleMotorID}, m_angleEncoder{angleEncoderID} {

	#pragma region // Drive motor configs

	m_driveMotor.ConfigFactoryDefault();

	m_driveMotor.SetNeutralMode(NeutralMode::Brake);

	m_driveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
	
	m_driveMotor.Config_kP(0, Swordtip::PIDF::Drive::P);
	m_driveMotor.Config_kI(0, Swordtip::PIDF::Drive::I);
	m_driveMotor.Config_kD(0, Swordtip::PIDF::Drive::D);
	m_driveMotor.Config_kF(0, Swordtip::PIDF::Drive::F);
	
	m_driveMotor.ConfigNominalOutputForward(0);
	m_driveMotor.ConfigNominalOutputReverse(0);
	m_driveMotor.ConfigPeakOutputForward(1);
	m_driveMotor.ConfigPeakOutputReverse(-1);

	#pragma endregion

	#pragma region // Angle motor configs

	m_angleMotor.ConfigFactoryDefault();

	m_angleMotor.SetSensorPhase(true);
	m_angleMotor.SetNeutralMode(NeutralMode::Brake);

	m_angleMotor.ConfigFeedbackNotContinuous(true);
	m_angleMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0);
	m_angleMotor.ConfigRemoteFeedbackFilter(angleEncoderID, RemoteSensorSource(13), 0, 0);
	m_angleMotor.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);
	
	m_angleMotor.Config_kP(0, Swordtip::PIDF::Angle::P);
	m_angleMotor.Config_kI(0, Swordtip::PIDF::Angle::I);
	m_angleMotor.Config_kD(0, Swordtip::PIDF::Angle::D);
	m_angleMotor.Config_kF(0, Swordtip::PIDF::Angle::F);
	m_angleMotor.Config_IntegralZone(0, 20);

	m_angleMotor.ConfigNominalOutputForward(0);
	m_angleMotor.ConfigNominalOutputReverse(0);
	m_angleMotor.ConfigPeakOutputForward(1);
	m_angleMotor.ConfigPeakOutputReverse(-1);

	#pragma endregion

	#pragma region // Encoder configs

	m_angleEncoder.ConfigMagnetOffset(magnetOffset);
	m_angleEncoder.SetPositionToAbsolute();

	m_angleEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
	
	#pragma endregion

	#pragma region // Misc configs for initilization

	m_angleMotor.Set(TalonFXControlMode::Position,0);

	CANID = angleEncoderID;

	#pragma endregion

}

frc::SwerveModulePosition SwerveModule::GetPosition() {
	auto distance = (((2 * M_PI * 2) / Swordtip::Misc::Drive_Gear_Ratio ) / 2048) * m_driveMotor.GetSelectedSensorPosition();
	auto angle = ((2 * M_PI) / 360) * m_angleEncoder.GetAbsolutePosition();
	return {units::meter_t{distance}, units::degree_t{angle}};
}

frc::Rotation2d SwerveModule::getAngle() {
	return frc::Rotation2d{units::degree_t{-m_angleEncoder.GetPosition()}};
}

frc::SwerveModuleState SwerveModule::Optimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle, bool optimize){
	return optimize ? frc::SwerveModuleState::Optimize(desiredState, getAngle()) : frc::SwerveModuleState{desiredState.speed,desiredState.angle};
	
	if(optimize){
		return frc::SwerveModuleState::Optimize(desiredState, getAngle());
	}else{
		return frc::SwerveModuleState{desiredState.speed,desiredState.angle};
	}
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {

	// Optimize the desired state to avoid angling the wheel further than 90 degrees in one go
	auto const [optimized_speed, optimized_angle] = Optimize(desiredState, getAngle(), true);

	// Module data to Dashboard
	switch (CANID){
	case 2:
		frc::SmartDashboard::PutNumber("Front Right angle", optimized_angle.Degrees().value());
		frc::SmartDashboard::PutNumber("Front Right speed", optimized_speed.value());
		break;
	case 5:
		frc::SmartDashboard::PutNumber("Rear Right angle", optimized_angle.Degrees().value());
		frc::SmartDashboard::PutNumber("Rear Right speed", optimized_speed.value());
		break;
	case 8:
		frc::SmartDashboard::PutNumber("Rear Left angle", optimized_angle.Degrees().value());
		frc::SmartDashboard::PutNumber("Rear Left speed", optimized_speed.value());
		break;
	case 11:
		frc::SmartDashboard::PutNumber("Front Left angle", optimized_angle.Degrees().value());
		frc::SmartDashboard::PutNumber("Front Left speed", optimized_speed.value());
		break;
	}

	// Set the motor outputs.
	m_driveMotor.Set((double) optimized_speed);
	m_angleMotor.Set(TalonFXControlMode::Position, optimized_angle.Degrees().value()*(Swordtip::Misc::Conversion_Factor));
}

void SwerveModule::SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode){
	m_driveMotor.SetNeutralMode(mode);
	m_angleMotor.SetNeutralMode(mode);
}