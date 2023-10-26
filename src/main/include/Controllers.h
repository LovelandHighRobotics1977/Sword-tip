#ifndef CONTROL_H
#define CONTROL_H

#include "Headers.h"

class Control {
	public:

	// Sets all variables

	class Drive {
	public:

		Drive(const int port):m_Joystick{port}{};

		bool field_oriented;

		bool gyro_reset;

		bool trigger_one;
		bool trigger_two;

		bool emergency_stop;

		double throttle;

		double forward;
		double strafe;
		double rotate;
		
		/**
		 * Update the controller variables 
		 * @attention Each control scheme is defined in this function
		 * @note Automaically chooses the control scheme based on the joystick name
		*/
		void update(){
			if(m_Joystick.GetName() == std::string{"HOTAS"}){
				field_oriented = !m_Joystick.GetRawButton(6);

				gyro_reset = m_Joystick.GetRawButton(2);

				trigger_one = m_Joystick.GetRawButton(1);
				trigger_two = m_Joystick.GetRawButton(15);

				emergency_stop = m_Joystick.GetRawButton(5);

				throttle = ((1 - ((m_Joystick.GetZ() + 1) / 2)));

				forward = m_Joystick.GetY(); // 1 is forward
				strafe = m_Joystick.GetX(); // 1 is left
				rotate = m_Joystick.GetRawAxis(5);
			}

			if(m_Joystick.GetName() == std::string{"Saitek X45"}){
				field_oriented = !m_Joystick.GetRawButton(7);

				gyro_reset = m_Joystick.GetRawButton(4);

				trigger_one = m_Joystick.GetRawButton(1);
				trigger_two = m_Joystick.GetRawButton(1);

				emergency_stop = m_Joystick.GetRawButton(8);

				throttle = ((1 - ((m_Joystick.GetRawAxis(4) + 1) / 2)));

				forward = m_Joystick.GetY(); // 1 is forward
				strafe = m_Joystick.GetX(); // 1 is left
				rotate = m_Joystick.GetRawAxis(3);
			}

			if(m_Joystick.GetName() == std::string{"Extreme 3D pro"}){
				field_oriented = !m_Joystick.GetRawButton(2);

				gyro_reset = m_Joystick.GetRawButton(3);

				trigger_one = m_Joystick.GetRawButton(1);
				trigger_two = m_Joystick.GetRawButton(1);

				emergency_stop = m_Joystick.GetRawButton(4);

				throttle = ((1 - ((m_Joystick.GetRawAxis(3) + 1) / 2)));

				forward = m_Joystick.GetY(); // 1 is forward
				strafe = m_Joystick.GetX(); // 1 is left
				rotate = m_Joystick.GetRawAxis(2);
			}
		
		}

		private:
			frc::Joystick m_Joystick;
	};

	class Mechanism {
	public:

		Mechanism(const int port):m_XboxController{port}{};

		bool angle_up;
		bool angle_down;

		bool intake_in;
		bool intake_out;

		bool speed_slow;
		bool speed_medium;
		bool speed_fast;

		void update(){
			angle_up = m_XboxController.GetLeftTriggerAxis() > 0;
			angle_down = m_XboxController.GetRightTriggerAxis() > 0;

			intake_in = m_XboxController.GetLeftBumper();
			intake_out = m_XboxController.GetRightBumper();

			speed_slow = m_XboxController.GetXButton();
			speed_medium = m_XboxController.GetYButton();
			speed_fast = m_XboxController.GetBButton();
		}

	private:
		frc::XboxController m_XboxController;

	};

	
};

#endif /* !CONTROL_H */