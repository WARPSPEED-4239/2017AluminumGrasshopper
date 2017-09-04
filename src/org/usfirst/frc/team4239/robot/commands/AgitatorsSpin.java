package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class AgitatorsSpin extends Command {

    public AgitatorsSpin() {
        requires(Robot.agitators);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.agitators.spin();
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
