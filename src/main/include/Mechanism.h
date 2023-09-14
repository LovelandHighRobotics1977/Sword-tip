// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Headers.h>

/**
 * Represents a mechanism
*/
class Mechanism {
    public:
        Mechanism();

        /**
			* Takes A Do and does Something with it
			*
			* @param Do The inputted Do
		*/
        void DoSomething(float up, float down);

    private:
        // Declare all mechanism variables, motors, sensors, ect here
        //declare x box controller
        frc::XboxController m_xboxController{1};
        //declare arm motors
        WPI_TalonFX m_armAngle{12};
        WPI_TalonFX m_roller{13};
        //declare limit switches for arm
        frc::DigitalInput m_LowerSwitch{0};
        frc::DigitalInput m_UpperSwitch{1};
        

        double Something;
};