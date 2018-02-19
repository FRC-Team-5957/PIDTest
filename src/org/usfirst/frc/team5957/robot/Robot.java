package org.usfirst.frc.team5957.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
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

	final double Kp = 0.05;
	final double Ki = 0.0;
	final double Kd = 0.0;
	final double delay = 0.004;
	
	double target = 90;

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
		// frontLeftPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// rearLeftPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// frontRightPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);
		// rearRightPID = new PIDController(Kp, Ki, Kd, gyro, frontLeft);

	}

	@Override
	public void robotPeriodic() {
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
		gyro.reset();
		time.start();

		double PCurrent = target - gyro.getAngle(); // 90 - 0
		double PLast = PCurrent; // 90
		double D = PLast - PCurrent; // 0
		double I = 0;
		double IIncrement = 0.01;
		
		if( (target < 0 && gyro.getAngle() > target - 30) || (target > 0 && gyro.getAngle() < target + 30)) {
			I = 0;
		} else {
			I = I + IIncrement;
		}
		double output = (PCurrent * Kp) + (I * Ki) - (D * Kd); // 0.9 + 0 - 0 

		while (output != 0 && gyro.getAngle() != target) {
 
			if (output > 0.6) {
				output = 0.6;
			} else if (output < -0.6) {
				output = -0.6;

			}
			System.out.print("P: " + PCurrent + " I: " + I + " D: " + D);
			System.out.print("Turn Speed: " + output); 
			base.arcadeDrive(0, -output); // TODO: reverse back to normal for new robot
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
	public void teleopInit() {
		gyro.reset();
		target = gyro.getAngle();
	}

	@Override
	public void teleopPeriodic() {
		double PCurrent = target - gyro.getAngle();
		double PLast = PCurrent;
		double D = PLast - PCurrent;
		double output = (PCurrent * Kp) - (D * Kd);

		// while (output != 0 && gyro.getAngle() != target) {
		
		 if (output > 0.6) {
		 output = 0.6;
		 } else if (output < -0.6) {
		 output = -0.6;
		
		 }
		
		// }

		if (joy.getRawAxis(4) < -0.05 || joy.getRawAxis(4) > 0.05) {
			base.arcadeDrive(joy.getRawAxis(1), -joy.getRawAxis(4), true);
			gyro.reset();
		} else {
			base.arcadeDrive(joy.getRawAxis(1), -output, true);
		}
		System.out.print("output: " + output);
		System.out.print("target: " + target);
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
