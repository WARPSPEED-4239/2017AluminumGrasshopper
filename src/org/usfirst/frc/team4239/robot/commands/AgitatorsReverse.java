package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class AgitatorsReverse extends Command {

    public AgitatorsReverse() {
        requires(Robot.agitators);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.agitators.reverse();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    	Robot.agitators.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
