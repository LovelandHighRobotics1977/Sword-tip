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
        void SetAngle(bool up, bool down);

        void SetIntake(bool in, bool out);

        int SetSpeed(bool x, bool y, bool b);

        int speed;

    private:
        // Declare all mechanism variables, motors, sensors, ect here

        double motorSpeed = 0;

        WPI_TalonFX m_AngleMotor{12};
        WPI_TalonFX m_IntakeMotor{13};

        frc::DigitalInput m_LowerSwitch{0};
        frc::DigitalInput m_UpperSwitch{1};
};