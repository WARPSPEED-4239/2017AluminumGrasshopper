package org.usfirst.frc.team4239.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
	public static final boolean DEBUG = true;
	
	public static final int 
		MOTOR_DRIVETRAIN_FRONT_LEFT = 0,
		MOTOR_DRIVETRAIN_FRONT_RIGHT = 1,
		MOTOR_DRIVETRAIN_REAR_LEFT = 2,
		MOTOR_DRIVETRAIN_REAR_RIGHT = 3,
		MOTOR_SUBFRAME = 4,
		MOTOR_CLIMBER = 5,
		MOTOR_FUEL_INTAKE = 6,
		MOTOR_SHOOTER = 8,
		MOTOR_HOPPER_AGITATOR = 7;
		
	
	public static final int
		LIMIT_SWITCH_SUBFRAME_TOP = 0,
		LIMIT_SWITCH_SUBFRAME_BOTTOM = 1,
		ENCODER_DRIVETRAIN_LEFT_A = 2,
		ENCODER_DRIVETRAIN_LEFT_B = 3,
		ENCODER_DRIVETRAIN_RIGHT_A = 4,
		ENCODER_DRIVETRAIN_RIGHT_B = 5;
		
	
	public static final int
		SOLENOID_GEAR_PICKUP_MECHANISM_LIFT_FORWARD = 6,
		SOLENOID_GEAR_PICKUP_MECHANISM_LIFT_REVERSE = 5,
		SOLENOID_GEAR_PICKUP_MECHANISM_CLAMP_FORWARD = 3,
		SOLENOID_GEAR_PICKUP_MECHANSIM_CLAMP_REVERSE = 4;
			                
}
