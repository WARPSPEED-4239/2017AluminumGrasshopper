package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.Robot;
import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.DrivetrainArcadeDrive;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import motion.control.MotionController;
import motion.control.MotionController.MotionOutput;
import motion.control.MotionController.MotionSource;

public class Drivetrain extends Subsystem {

	private RobotDrive drive;
	private ADXRS450_Gyro gyro;
	private Encoder leftEncoder, rightEncoder;
	
	private MotionController distanceController;
	private MotionController angleController;
	
	public Drivetrain() {
		super();
		
		drive = new RobotDrive(RobotMap.MOTOR_DRIVETRAIN_FRONT_LEFT,
				               RobotMap.MOTOR_DRIVETRAIN_REAR_LEFT,
				               RobotMap.MOTOR_DRIVETRAIN_FRONT_RIGHT,
				               RobotMap.MOTOR_DRIVETRAIN_REAR_RIGHT);
		
		/*
		 * Gyro gives angle in degrees where
		 * 		negative numbers = left turns
		 *   	positive numbers = right turns
		 */
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		
		/*
		 * Encoders give distance in feet where
		 * 		negative numbers = traveled backwards
		 * 		positive numbers = traveled forwards
		 */
		leftEncoder = new Encoder(RobotMap.ENCODER_DRIVETRAIN_LEFT_A,
				                  RobotMap.ENCODER_DRIVETRAIN_LEFT_B,
				                  true);
		rightEncoder = new Encoder(RobotMap.ENCODER_DRIVETRAIN_RIGHT_A,
				                   RobotMap.ENCODER_DRIVETRAIN_RIGHT_B,
				                   false);
		
		final double WHEEL_DIAMETER = 0.5;                                // 6in. = .5ft.
		final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
		final double PULSES_PER_REVOLUTION = 1440; 
		
		leftEncoder.setDistancePerPulse(4 * WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION);
		rightEncoder.setDistancePerPulse(4 * WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION);
		
		/*
		 * Setup PID controller to be used in autonomous
		 */
		distanceController = buildDistanceMotionController();
		angleController = buildAngleMotionController();
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new DrivetrainArcadeDrive());
	}
	
	public void stop() {
		drive.stopMotor();
	}
	
	public double getLinearVelocity() {
		double leftVelocity = leftEncoder.getRate();
		double rightVelocity = rightEncoder.getRate();
		return (leftVelocity + rightVelocity) / 2;
	}
	
	/*
	 * 1) Calculates move and rotate values from XboxController
	 * 2) Adds dead-space to zero near values that are near zero
	 * 3) Throttles back speed when A button is pressed
	 */
	public void arcadeDrive(XboxController controller) {
		usingGyro = false;
		
		double move = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
		double rotate = controller.getX(Hand.kLeft);
		
		final double MIN_MOVE_THRESHOLD = 0.05;
		final double MIN_ROTATE_THRESHOLD = 0.20;
		if (Math.abs(move) < MIN_MOVE_THRESHOLD)
			move = 0.0;
		if (Math.abs(rotate) < MIN_ROTATE_THRESHOLD)
			rotate = 0.0;
		
		if (Robot.oi.getController().getAButton()) {
			move *= 0.8;
		}
		
		arcadeDrive(move, rotate);
	}
	
	private boolean straightDrive = false;
	private boolean usingGyro = false;
	
	public void arcadeDriveGyroAssist(XboxController controller) {
		usingGyro = true;
		
		double move = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
		double rotate = controller.getX(Hand.kLeft);
		
		final double MIN_MOVE_THRESHOLD = 0.05;
		final double MIN_ROTATE_THRESHOLD = 0.20;
		if (Math.abs(move) < MIN_MOVE_THRESHOLD)
			move = 0.0;
		if (Math.abs(rotate) < MIN_ROTATE_THRESHOLD) {
			if (!straightDrive) {
				gyro.reset();
				straightDrive = true;
			}
			final double Kp = 0.05;
			rotate = -Kp * gyro.getAngle();
		}
		else {
			straightDrive = false;
		}
		
		SmartDashboard.putBoolean("Drive Straight", straightDrive);
		
		if (Robot.oi.getController().getAButton()) {
			move *= 0.8;
		}
		
		arcadeDrive(move, rotate);
	}
	
	/*
	 * move: 
	 *       -1 = backward
	 *        0 = stop
	 *        1 = forward
	 * rotate:
	 *       -1 = left
	 *        0 = straight
	 *        1 = right
	 */
	public void arcadeDrive(double move, double rotate) {
		
		SmartDashboard.putBoolean("Using Gyro", usingGyro);
		
		if (RobotMap.DEBUG) {
			SmartDashboard.putNumber("Move", move);
			SmartDashboard.putNumber("Rotate", rotate);
			SmartDashboard.putNumber("Heading", gyro.getAngle());
			SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
			SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
		}
		
		/*
		 * The WPILib convention has the rotate value negated.
		 */
		drive.arcadeDrive(move, -rotate);
	}
	
	/*
	 * Zero the encoders and gyro
	 */
	public void resetSensors() {
		leftEncoder.reset();
		rightEncoder.reset();
		gyro.reset();
	}
	
	public MotionController getDistanceMotionController() {
		return distanceController;
	}
	
	public MotionController getAngleMotionController() {
		return angleController;
	}
	
	private MotionController buildDistanceMotionController() {
		// PID constants
		final double Kp = 0.0;
		final double Ki = 0.0;
		final double Kd = 0.0;
		final double Kv = 0.0;
		final double Ka = 0.0;
		
		MotionSource distanceSource = new MotionSource() {
			@Override
			public double getPosition() {
				double leftDistance = leftEncoder.getDistance();
				double rightDistance = rightEncoder.getDistance();
				return (leftDistance + rightDistance) / 2;
			}
		};
		
		MotionOutput distanceOutput = new MotionOutput() {
			@Override
			public void updateMotors(double output) {
				final double Kp = 0.15;
				double rotate = -Kp * gyro.getAngle();
				arcadeDrive(output, rotate);
			}
		};
		
		MotionController controller = new MotionController(distanceSource, distanceOutput, Kp, Ki, Kd, Kv, Ka);
		controller.setTolerance(1 / 12);
		return controller;
	}
	
	private MotionController buildAngleMotionController() {
		final double Kp = 0.0;
		final double Ki = 0.0;
		final double Kd = 0.0;
		final double Kv = 0.0;
		final double Ka = 0.0;
		
		MotionSource angleSource = new MotionSource() {
			@Override
			public double getPosition() {
				return gyro.getAngle();
			}
		};
		
		MotionOutput angleOutput = new MotionOutput() {
			@Override
			public void updateMotors(double output) {
				arcadeDrive(0.0, output);
			}
		};
		
		MotionController controller = new MotionController(angleSource, angleOutput, Kp, Ki, Kd, Kv, Ka);
		controller.setTolerance(1.0);
		return controller;
	}
}
