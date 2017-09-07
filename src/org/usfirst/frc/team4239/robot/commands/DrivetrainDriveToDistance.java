package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainDriveToDistance extends Command {

	private PIDController distanceController;
	private double setpoint;
	
	/*
	 * distance: the distance to drive in feet.
	 * 		positive = forward
	 * 		negative = backward
	 */
	
    public DrivetrainDriveToDistance(double distance) {
        requires(Robot.drivetrain);
        setpoint = distance;
        distanceController = Robot.drivetrain.getDistancePIDController();
    }

    protected void initialize() {
    	Robot.drivetrain.resetSensors();
        distanceController.setSetpoint(setpoint);
    	distanceController.enable();
    }

    protected void execute() {
    	SmartDashboard.putNumber("Distance Error", distanceController.getError());
    }

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
