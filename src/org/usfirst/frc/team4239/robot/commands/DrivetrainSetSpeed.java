package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DrivetrainSetSpeed extends Command {

	private double move, rotate, timeout;
	
    public DrivetrainSetSpeed(double move, double rotate, double timeout) {
        requires(Robot.drivetrain);
        
        this.move = move;
        this.rotate = rotate;
        this.timeout = timeout;
    }

    protected void initialize() {
    	setTimeout(timeout);
    }

    protected void execute() {
    	Robot.drivetrain.arcadeDrive(move, rotate);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
    	Robot.drivetrain.stop();
    }

    protected void interrupted() {
    	end();
    }
}
