package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainRotateToAngle extends Command {

	private PIDController angleController;
	private double setpoint;
	
	/*
	 * angle: the angle to rotate in degrees
	 * 		positive = right
	 * 		negative = left
	 */
    public DrivetrainRotateToAngle(double angle) {
        requires(Robot.drivetrain);
        setpoint = angle;
        angleController = Robot.drivetrain.getAnglePIDController();
    }

    protected void initialize() {
    	Robot.drivetrain.resetSensors();
        angleController.setSetpoint(setpoint);
    	angleController.enable();
    }

    protected void execute() {
    	SmartDashboard.putNumber("Rotate Error", angleController.getError());
    }

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
