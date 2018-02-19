package org.usfirst.frc.team5957.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

	DifferentialDrive base;
	SpeedController frontLeft;
	SpeedController rearLeft;
	SpeedController frontRight;
	SpeedController rearRight;
	SpeedControllerGroup left;
	SpeedControllerGroup right;
	Joystick joy;
	ADXRS450_Gyro gyro;
	Timer time;
	// PIDController frontLeftPID;
	// PIDController rearLeftPID;
	// PIDController frontRightPID;
	// PIDController rearRightPID;

	final double Kp = 0.2125;
	final double Kd = 0.4; // Kd = 6.4 when Kp = 0.3 can do 90 degrees turn
	final double delay = 0.004;

	double target = 0;

	double PCurrent = 0; // 90 - 0
	double PLast = 0; // 90
	double D = 0; // 0
	double output = 0; // 0.9 - 0

	double done = 0;

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

		left = new SpeedControllerGroup(frontLeft, rearLeft);
		right = new SpeedControllerGroup(frontRight, rearRight);

		base = new DifferentialDrive(left, right);
		joy = new Joystick(0);
		gyro = new ADXRS450_Gyro();
		time = new Timer();
		gyro.reset();
		gyro.calibrate();
		// frontLeftPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// rearLeftPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// frontRightPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// rearRightPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);

	}

	@Override
	public void robotPeriodic() {
		PCurrent = target - gyro.getAngle(); // 90 - 0
		PLast = PCurrent; // 90
		D = PLast - PCurrent; // 0
		output = (PCurrent * Kp) - (D * Kd); // 0.9 - 0
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousInit() {
		time.reset();
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
			System.out.print(-output);
			base.arcadeDrive(0, -output);
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

	private void drive(double power) {
		target = gyro.getAngle();
		base.arcadeDrive(power, -output);
	}

	private void stop() {
		target = gyro.getAngle();
		base.arcadeDrive(0, -output);
	}

	private void turnToAngle(double angle) {
		target = angle;

	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		gyro.reset();
		gyro.calibrate();

	}

	@Override
	public void testPeriodic() {
		if (output != 0 && gyro.getAngle() != target) {

			if (output > 0.6) {
				output = 0.6;
			} else if (output < -0.6) {
				output = -0.6;

			}
			System.out.print(-output);
			Timer.delay(delay);
			PLast = PCurrent;
			PCurrent = target - gyro.getAngle();
			D = PLast - PCurrent;
			output = (PCurrent * Kp) - (D * Kd);

		}
		if (time.get() > 0.01 && time.get() < 1) {
			drive(0.4);
		} else if (time.get() > 1.01 && time.get() < 1.05) {
			stop();
		} else if (time.get() > 1.06 && done == 0) {
			target = target + 90;
			done = 1;
		}

	}

}
