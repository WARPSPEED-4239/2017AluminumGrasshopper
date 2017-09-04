package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;


public class ShooterStop extends Command {

    public ShooterStop() {
        requires(Robot.shooter);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.shooter.stop();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {}

    protected void interrupted() {}
    
}
