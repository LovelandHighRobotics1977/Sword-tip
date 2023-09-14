#include "Mechanism.h"

Mechanism::Mechanism(){
	// Initial Mechanism Functions
	if(!m_UpperSwitch.Get())
	{
		m_armAngle.Set(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		m_armAngle.Set(0);
	}
	
}

void Mechanism::DoSomething(float up, float down){
	// Mechanism Action Function
	if (m_xboxController.GetRightBumper())
	{
		m_roller.Set(-1);
	}
	else if(m_xboxController.GetLeftBumper())
	{
		m_roller.Set(1);
	}
	if (!m_LowerSwitch.Get())
	{
		m_armAngle.Set(down);
	}
	if(!m_UpperSwitch.Get())
	{
		m_armAngle.Set(up);
	}
}