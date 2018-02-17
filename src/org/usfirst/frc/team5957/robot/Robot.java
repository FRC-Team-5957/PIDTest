package org.usfirst.frc.team5957.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

public class Robot extends IterativeRobot {

	RobotDrive base;
	SpeedController frontLeft;
	SpeedController rearLeft;
	SpeedController frontRight;
	SpeedController rearRight;
	Joystick joy;
	ADXRS450_Gyro gyro;
	Timer time;
	// PIDController frontLeftPID;
	// PIDController rearLeftPID;
	// PIDController frontRightPID;
	// PIDController rearRightPID;

	final double Kp = 0.05;
	final double Ki = 0;
	final double Kd = 0.0275;
	final double delay = 0.004;

	@Override
	public void robotInit() {

		// Front left, rear left, front right, rear right
		frontLeft = new VictorSP(2);
		frontLeft.setInverted(true);

		rearLeft = new VictorSP(3);
		rearLeft.setInverted(true);

		frontRight = new VictorSP(0);
		frontRight.setInverted(true);

		rearRight = new VictorSP(1);
		rearRight.setInverted(true);

		base = new RobotDrive(rearLeft, frontLeft, rearRight, frontRight);
		joy = new Joystick(0);
		gyro = new ADXRS450_Gyro();
		time = new Timer();
		// frontLeftPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// rearLeftPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// frontRightPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// rearRightPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);

	}

	@Override
	public void autonomousInit() {
		time.reset();
		gyro.reset();
		time.start();

		double target = 90;

		double PCurrent = target - gyro.getAngle(); // 90 - 0
		double PLast = PCurrent; // 90
		double D = PLast - PCurrent; // 0
		double output = (PCurrent * Kp) - (D * Kd); // 0.9 - 0

		while (output != 0 && gyro.getAngle() != target) {

			if (output > 0.6) {
				output = 0.6;
			} else if (output < -0.6) {
				output = -0.6;

			}
			base.arcadeDrive(0, output);
			Timer.delay(delay);
			PLast = PCurrent;
			PCurrent = target - gyro.getAngle();
			D = PLast - PCurrent;
			output = (PCurrent * Kp) - (D * Kd);

		}

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopPeriodic() {

		// base.arcadeDrive(joy.getRawAxis(1), joy.getRawAxis(4));
		if (joy.getRawButton(1)) {

			frontLeft.set(0.5);
			Timer.delay(delay);

		} else if (joy.getRawButton(2)) {

			rearLeft.set(0.5);
			Timer.delay(delay);

		} else if (joy.getRawButton(3)) {

			frontRight.set(0.5);
			Timer.delay(delay);

		} else if (joy.getRawButton(4)) {

			rearRight.set(0.5);
			Timer.delay(delay);

		}

	}

	// public double getOutput(double angle) {
	// double output;
	//
	// return output;
	//
	// }

	@Override
	public void testPeriodic() {
	}

	// public void setDrivePID(double angle) {
	//
	// frontLeftPID.enable();
	// rearLeftPID.enable();
	// frontRightPID.enable();
	// rearRightPID.enable();
	// frontLeftPID.setSetpoint(angle);
	// rearLeftPID.setSetpoint(angle);
	// frontRightPID.setSetpoint(angle);
	// rearRightPID.setSetpoint(angle);
	//
	// }
}