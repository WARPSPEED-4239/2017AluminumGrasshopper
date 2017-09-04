package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DrivetrainArcadeDriveGyroAssist extends Command {

    public DrivetrainArcadeDriveGyroAssist() {
        requires(Robot.drivetrain);
    }

    protected void initialize() {
    	Robot.drivetrain.resetSensors();
    }

    protected void execute() {
    	Robot.drivetrain.arcadeDriveGyroAssist(Robot.oi.getController());
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {}

    protected void interrupted() {}
    
}
