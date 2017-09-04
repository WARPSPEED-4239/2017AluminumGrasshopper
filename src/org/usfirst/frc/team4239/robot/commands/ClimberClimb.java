package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberClimb extends Command {

    public ClimberClimb() {
        requires(Robot.climber);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.climber.climb();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    	Robot.climber.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
