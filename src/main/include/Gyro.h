#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "AHRS.h"

class Gyro {
	private:
		inline static Gyro* instance{nullptr};
		Gyro() = default;
		~Gyro() = default;
	
	public:
	Gyro(const Gyro&) = delete;
	Gyro& operator=(const Gyro&) = delete;

	static Gyro* GetInstance(){
		if ( !instance ){
			instance = new Gyro();
		}
		return instance;
	}

	AHRS ahrs{frc::SerialPort::Port::kUSB1};

	/**
	 * Return the heading of the robot as a Rotation2d. The angle is continuous, that is it will continue from 360 to 361 degrees. This allows algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from 360 to 0 on the second time around. The angle is expected to increase as the gyro turns counterclockwise when looked at from the top. It needs to follow the NWU axis convention.
	 * @returns
	 * the current heading of the robot as a Rotation2d. This heading is based on integration of the returned rate from the gyro.
	*/
	frc::Rotation2d GetRotation2d(){
		return frc::Rotation2d{units::degree_t{ahrs.GetYaw()}};
	}

	/**
	 * Reset the Yaw gyro.
	 * 
	 * Resets the Gyro Z (Yaw) axis to a heading of zero. This can be used if 
	 * there is significant drift in the gyro and it needs to be recalibrated 
	 * after it has been running.
	*/
	void Reset(){
		ahrs.Reset();
	}

	/**
	 * Sets the user-specified yaw offset to the current
	 * yaw value reported by the sensor.
	 * 
	 * This user-specified yaw offset is automatically
	 * subtracted from subsequent yaw values reported by
	 * the getYaw() method.
	 * 
	 * NOTE: This method has no effect if the sensor is
	 * currently calibrating, since resetting the yaw will
	 * interfere with the calibration process.
	*/
	void ZeroYaw(){
		ahrs.ZeroYaw();
	}

	double GetYaw(){
		return ahrs.GetYaw();
	}
};

#endif /* !GYROSCOPE_H */