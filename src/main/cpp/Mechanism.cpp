#include "Mechanism.h"

Mechanism::Mechanism(){
	m_AngleMotor.SetNeutralMode(Brake);
	m_IntakeMotor.SetNeutralMode(Coast);
}

void Mechanism::SetAngle(bool up, bool down){
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
	
	if(out && !in){
		m_IntakeMotor.Set(-motorSpeed);
	}else if(in && !out){
		m_IntakeMotor.Set(0.2);
	}else{
		m_IntakeMotor.Set(0);
	}
}