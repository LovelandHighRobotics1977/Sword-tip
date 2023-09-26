#include "Mechanism.h"

Mechanism::Mechanism(){
	m_AngleMotor.SetNeutralMode(Brake);
	m_IntakeMotor.SetNeutralMode(Coast);
}

void Mechanism::SetAngle(bool up, bool down){
	if(m_UpperSwitch.Get()){
		frc::SmartDashboard::PutString("up","  ");
	}else{
		frc::SmartDashboard::PutString("up","██");
	}

	if(m_LowerSwitch.Get()){
		frc::SmartDashboard::PutString("down","  ");
	}else{
		frc::SmartDashboard::PutString("down","██");
	}


	if(up && !down){
		if(m_UpperSwitch.Get()){
			m_AngleMotor.Set( 0.2 );
		}
	}else if(down && !up){
		if(m_LowerSwitch.Get()){
			m_AngleMotor.Set( -0.2 );
		}
	}else{
		m_AngleMotor.Set( 0 );
	}
}

void Mechanism::SetIntake(bool in, bool out, bool low, bool mid, bool high){
	
	motorSpeed = ((low * 0.2) + (mid * 0.4) + (high * 0.9));
	if(low){
		frc::SmartDashboard::PutString("low","██");
	}else{
		frc::SmartDashboard::PutString("low","  ");
	}

	if(mid){
		frc::SmartDashboard::PutString("mid","██");
	}else{
		frc::SmartDashboard::PutString("mid","  ");
	}

	if(high){
		frc::SmartDashboard::PutString("hgh","██");
	}else{
		frc::SmartDashboard::PutString("hgh","  ");
	}
	
	if(out && !in){
		m_IntakeMotor.Set(-motorSpeed);
	}else if(in && !out){
		m_IntakeMotor.Set(0.2);
	}else{
		m_IntakeMotor.Set(0);
	}
}