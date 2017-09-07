package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import motion.control.MotionController;

public class DrivetrainRotateToAngle extends Command {

	private MotionController angleController;
	private double setpoint;
	
	/*
	 * angle: the angle to rotate in degrees
	 * 		positive = right
	 * 		negative = left
	 */
    public DrivetrainRotateToAngle(double angle) {
        requires(Robot.drivetrain);
        setpoint = angle;
        angleController = Robot.drivetrain.getAngleMotionController();
    }

    protected void initialize() {
    	Robot.drivetrain.resetSensors();
    	final double targetPosition = setpoint / 180 * Math.PI;
    	final double cruisingVelocity = 0.25; // 0.25 radians / sec
    	final double acceleration = 0.05;	   // 0.05 radians / sec^2
        angleController.setMotionProfile(targetPosition, cruisingVelocity, acceleration);
    	angleController.enable();
    }

    protected void execute() {}

    protected boolean isFinished() {
        return angleController.onTarget();
    }

    protected void end() {
    	angleController.disable();
    	Robot.drivetrain.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
