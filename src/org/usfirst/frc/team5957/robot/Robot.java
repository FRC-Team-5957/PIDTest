package org.usfirst.frc.team5957.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
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
	Encoder leftE;
	// PIDController frontLeftPID;
	// PIDController rearLeftPID;
	// PIDController frontRightPID;
	// PIDController rearRightPID;

	final double Kp = 0.2125;
	final double Kd = 3.2; // Kd = 3.2 when Kp = 0.2125 can do 90 degrees turn

	final double eKp = 0.0;
	final double eKd = 0.0;
	final double delay = 0.004;

	double gyroTarget = 0;
	double encTarget = 0;

	double gyroP = 0; // 90 - 0
	double gyroPLast = 0; // 90
	double gyroD = 0; // 0
	double output = 0; // 0.9 - 0
	double gyroOut = 0;

	double encP = 0;
	double encPLast = 0;
	double encD = 0;
	double encOut = 0;

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

		leftE = new Encoder(2, 3, false);

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
		gyroP = gyroTarget - gyro.getAngle(); // 90 - 0
		gyroPLast = gyroP; // 90
		gyroD = gyroPLast - gyroP; // 0
		output = (gyroP * Kp) - (gyroD * Kd); // 0.9 - 0
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	boolean straightAuto = true;

	@Override
	public void autonomousInit() {
		time.reset();
		time.start();
		leftE.reset();
		if (straightAuto == false) { // turn to 90 degrees
			double target = 90;
			double gyroP = target - gyro.getAngle(); // 90 - 0
			double gyroPLast = gyroP; // 90
			double gyroD = gyroPLast - gyroP; // 0
			double output = (gyroP * Kp) - (gyroD * Kd); // 0.9 - 0

			while (output != 0 && gyro.getAngle() != target) {
				if (output > 0.6) {
					output = 0.6;
				} else if (output < -0.6) {
					output = -0.6;

				}
				base.arcadeDrive(0, -output);
				Timer.delay(delay);
				gyroPLast = gyroP;
				gyroP = target - gyro.getAngle();
				gyroD = gyroPLast - gyroP;
				output = (gyroP * Kp) - (gyroD * Kd);
			}
		} else { // drive for 2 meters (eventually)
			gyroPID();
			encPID();
			encTarget = 4600; // ~2 yards
			base.arcadeDrive(encOut, -gyroOut);
		}

	}

	@Override
	public void autonomousPeriodic() {
		System.out.println(leftE.get());
		if (leftE.get() > 5000) {
			base.arcadeDrive(0, -output);
		}
	}

	private void turn(double angle) {
		gyroTarget = angle;

	}

	private void move(double dist) {
		encTarget = dist;
	}

	private void gyroPID() {
		gyroP = gyroTarget - gyro.getAngle(); // 90 - 0
		gyroPLast = gyroP; // 90
		gyroD = gyroPLast - gyroP; // 0
		gyroOut = (gyroP * Kp) - (gyroD * Kd); // 0.9 - 0
		if (gyroOut != 0 && gyro.getAngle() != gyroTarget) {
			if (gyroOut > 0.6) {
				gyroOut = 0.6;
			} else if (gyroOut < -0.6) {
				gyroOut = -0.6;

			}
			Timer.delay(delay);
			gyroPLast = gyroP;
			gyroP = gyroTarget - gyro.getAngle();
			gyroD = gyroPLast - gyroP;
			gyroOut = (gyroP * Kp) - (gyroD * Kd);
		}
	}

	private void encPID() {
		encP = encTarget - leftE.get();
		encPLast = encP;
		encD = encPLast - encP;
		encOut = (encP * eKp) - (encD * eKd);
		if (encOut != 0 && leftE.get() != encTarget)
			if (encOut > 0.6) {
				encOut = 0.6;
			} else if (encOut < -0.6) {
				encOut = -0.6;
			}
		Timer.delay(delay);
		encPLast = encP;
		encP = encTarget - leftE.get();
		encD = encPLast - encP;
		encOut = (encP * eKp) - (encD * eKd);
	}

	@Override
	public void teleopInit() {

	}

	@Override
	public void teleopPeriodic() {
		base.arcadeDrive(joy.getRawAxis(1), -joy.getRawAxis(4));
		System.out.print(leftE.getDistance());
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

}
