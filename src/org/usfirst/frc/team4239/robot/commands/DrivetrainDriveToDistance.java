package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import motion.control.MotionController;

public class DrivetrainDriveToDistance extends Command {

	private MotionController distanceController;
	private double setpoint;
	
	/*
	 * distance: the distance to drive in feet.
	 * 		positive = forward
	 * 		negative = backward
	 */
	
    public DrivetrainDriveToDistance(double distance) {
        requires(Robot.drivetrain);
        setpoint = distance;
        distanceController = Robot.drivetrain.getDistanceMotionController();
    }

    protected void initialize() {
    	Robot.drivetrain.resetSensors();
    	final double targetPosition = setpoint;
    	final double cruisingVelocity = 5.0; // 5 feet / sec
    	final double acceleration = 1.0;     // 1 feet / sec^2
        distanceController.setMotionProfile(targetPosition, cruisingVelocity, acceleration);
    	distanceController.enable();
    }

    protected void execute() {}

    protected boolean isFinished() {
        return distanceController.onTarget();
    }

    protected void end() {
    	distanceController.disable();
    	Robot.drivetrain.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
