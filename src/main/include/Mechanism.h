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
			* Takes two boolean values and sets the angle of the mechanism arm
			*
			* @param up Angle the mechanism up if true
            * @param down Angle the mechanism down if true
		*/
        void SetAngle(bool up, bool down);

        /**
			* Takes two boolean values and controls the intake
			*
			* @param in Set the mechanism to intake a game piece
            * @param out Set the mechanism to fire a game piece
		*/
        void SetIntake(bool in, bool out);

        /**
			* Takes three boolean values and sets firing speed
			*
			* @param slow Set the firing speed to slow
            * @param medium Set the firing speed to medium
            * @param fast Set the firing speed to maximum
            * 
            * 
            * @note
            * Slow -> Best for low nodes
            * @note
            * Medium -> Best for middle nodes
            * @note
            * Fast -> Best for high nodes (more challenging to hit)
		*/
        void SetSpeed(ArmData target);

        int speed = 0;

    private:
        // Declare all mechanism variables, motors, sensors, ect here

        double motorSpeed = 0;

        WPI_TalonFX m_AngleMotor{12};
        WPI_TalonFX m_IntakeMotor{13};

        frc::DigitalInput m_LowerSwitch{0};
        frc::DigitalInput m_UpperSwitch{1};
};