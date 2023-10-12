#include "Mechanism.h"

Mechanism::Mechanism(){
	m_AngleMotor.SetNeutralMode(Brake);
	m_IntakeMotor.SetNeutralMode(Brake);
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

void Mechanism::SetIntake(bool in, bool out){
	switch(speed){
		case 1:
			motorSpeed = 0.2;
			break;
		case 2:
			motorSpeed = 0.4;
			break;
		case 3:
			motorSpeed = 0.9;
			break;
	}

	if(speed == 1){
		frc::SmartDashboard::PutString("low","██");
	}else{
		frc::SmartDashboard::PutString("low","  ");
	}

	if(speed == 2){
		frc::SmartDashboard::PutString("mid","██");
	}else{
		frc::SmartDashboard::PutString("mid","  ");
	}

	if(speed == 3){
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
void Mechanism::SetSpeed(bool slow, bool medium, bool fast){
	if(slow){
		speed = 1;
	}
	else if(medium){
		speed = 2;
	}
	else if(fast){
		speed = 3;
	}
}