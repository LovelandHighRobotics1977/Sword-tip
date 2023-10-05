#ifndef CONTROL_H
#define CONTROL_H

#include "Headers.h"

frc::Joystick m_Joystick{0};
frc::XboxController m_Xbox{1};

class Control {
	private:
		
	
	public:

// Sets all variables

	class Drive {
	public:
		bool field_oriented;

		bool gyro_reset;

		bool trigger_one;
		bool trigger_two;

		bool emergency_stop;

		double throttle;

		double forward;
		double strafe;
		double rotate;

		void update(){
			field_oriented = !m_Joystick.GetRawButton(6);

			gyro_reset = m_Joystick.GetRawButton(2);

			trigger_one = m_Joystick.GetRawButton(1);
			trigger_two = m_Joystick.GetRawButton(15);

			emergency_stop = m_Joystick.GetRawButton(5);

			throttle = ((1 - ((m_Joystick.GetZ() + 1) / 2)));

			forward = m_Joystick.GetY();
			strafe = -m_Joystick.GetX();
			rotate = m_Joystick.GetRawAxis(5);
		}
	};

	class Mechanism {
	public:
		bool angle_up;
		bool angle_down;

		bool intake_in;
		bool intake_out;

		bool speed_slow;
		bool speed_medium;
		bool speed_fast;

		void update(){
			angle_up = m_Xbox.GetLeftTriggerAxis() > 0;
			angle_down = m_Xbox.GetRightTriggerAxis() > 0;

			intake_in = m_Xbox.GetLeftBumper();
			intake_out = m_Xbox.GetRightBumper();

			speed_slow = m_Xbox.GetXButton();
			speed_medium = m_Xbox.GetYButton();
			speed_fast = m_Xbox.GetBButton();
		}
	};
};

#endif /* !CONTROL_H */