package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.Robot;
import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.DrivetrainArcadeDrive;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import motion.control.MotionOutput;
import motion.control.MotionSource;

public class Drivetrain extends Subsystem {

	private RobotDrive drive;
	private ADXRS450_Gyro gyro;
	private Encoder leftEncoder, rightEncoder;
	
	private PIDController distanceController;
	private PIDController angleController;
	
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
		distanceController = buildDistancePIDController();
		angleController = buildAnglePIDController();

		/*
		if (RobotMap.DEBUG) {
			try {
				SmartDashboard.putData("Distance PID", distanceController);
				SmartDashboard.putData("Angle PID", angleController);
			}
			catch (Exception e) {
				SmartDashboard.putString("Error Message", e.getMessage());
			}
		}
		*/
		
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new DrivetrainArcadeDrive());
	}
	
	public void stop() {
		drive.stopMotor();
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
	
	public PIDController getDistancePIDController() {
		return distanceController;
	}
	
	public PIDController getAnglePIDController() {
		return angleController;
	}
	
	private PIDController buildDistancePIDController() {
		// PID constants
		final double Kp = 0.22;
		final double Ki = 0.0;
		final double Kd = 0.5;
		
		// Ensure the robot always moves unless OnTarget
		final double MIN_SPEED = 0.35;
		
		// Stop the robot from moving too fast
		final double MAX_SPEED = 0.50;
		
		// Consider the robot OnTarget if it's measured error is within +/- TOLERANCE
		final double TOLERANCE = 1 / 12;
		
		PIDSource distanceSource = new PIDSource() {
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {}
			
			@Override
			public double pidGet() {
				double leftDistance = leftEncoder.getDistance();
				double rightDistance = rightEncoder.getDistance();
				
				return (leftDistance + rightDistance) / 2;
			}
			
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		
		PIDOutput distanceOutput = new PIDOutput() {
			@Override
			public void pidWrite(double output) {
				/*
				if (RobotMap.DEBUG) {
					SmartDashboard.putNumber("Distance Error", distanceController.getError());
					SmartDashboard.putBoolean("Distance OnTarget", distanceController.onTarget());
				}
				*/
				
				// Apply the speed clamp
				if (output > 0 && output > MAX_SPEED)
					output = MAX_SPEED;
				else if (output > 0 && output < MIN_SPEED)
					output = MIN_SPEED;
				else if (output < 0 && output < -MAX_SPEED)
					output = -MAX_SPEED;
				else if (output < 0 && output > -MIN_SPEED)
					output = -MIN_SPEED;
				
				/*
				 * Read the gyro to track how far we have veered off angle.
				 * Use simple P-controller to straighten out.
				 * Example:
				 *    gyro.getAngle() = 2
				 *    rotate = -0.15 * 5
				 *           = -0.25
				 *    So, we rotate to the left with speed 0.25       
				 */
				final double Kp = 0.15;
				double rotate = -Kp * gyro.getAngle();
				
				arcadeDrive(output, rotate);
			}
		};
		
		PIDController controller = new PIDController(Kp, Ki, Kd, distanceSource, distanceOutput);
		controller.setAbsoluteTolerance(TOLERANCE);
		return controller;
	}
	
	private PIDController buildAnglePIDController() {
		// PID constants
		final double Kp = 0.05;
		final double Ki = 0.0;
		final double Kd = 0.01;
		
		// Ensure the robot always moves unless OnTarget
		final double MIN_SPEED = 0.45;
		
		// Stop the robot from moving too fast
		final double MAX_SPEED = 0.7;
		
		// Consider the robot OnTarget iff its measured error is within +/- TOLERANCE
		final double TOLERANCE = 1.00;
		
		PIDSource angleSource = new PIDSource() {
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {				}
			
			@Override
			public double pidGet() {
				return gyro.getAngle();
			}
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		
		PIDOutput angleOutput = new PIDOutput() {
			@Override
			public void pidWrite(double output) {
				/*
				if (RobotMap.DEBUG) {
					SmartDashboard.putNumber("Angle Error", angleController.getError());
					SmartDashboard.putBoolean("Angle OnTarget", angleController.onTarget());
				}
				*/

				// Apply the speed clamp
				if (output > 0 && output > MAX_SPEED)
					output = MAX_SPEED;
				else if (output > 0 && output < MIN_SPEED)
					output = MIN_SPEED;
				else if (output < 0 && output < -MAX_SPEED)
					output = -MAX_SPEED;
				else if (output < 0 && output > -MIN_SPEED)
					output = -MIN_SPEED;
				
				arcadeDrive(0.0, output);
			}
		};
		
		PIDController controller = new PIDController(Kp, Ki, Kd, angleSource, angleOutput);
		controller.setAbsoluteTolerance(TOLERANCE);
		controller.setContinuous(true);
		return controller;
	}
	
	public double getDistance() {
		double leftDistance = leftEncoder.getDistance();
		double rightDistance = rightEncoder.getDistance();
		return (leftDistance + rightDistance) / 2;
	}
	
	public double getRate() {
		double leftRate = leftEncoder.getRate();
		double rightRate = rightEncoder.getRate();
		return (leftRate + rightRate) / 2;
	}
	
	public MotionSource getDistanceSource() {
		return new MotionSource() {
			@Override
			public double motionGet() {
				return getDistance();
			}
		};
	}
	
	public MotionOutput getDistanceOutput() {
		return new MotionOutput() {			
			@Override
			public void updateMotors(double output) {
				final double Kp = 0.0;
				double rotate = -Kp * gyro.getAngle();
				
				drive.arcadeDrive(output, rotate);
			}
		};
	}
}
