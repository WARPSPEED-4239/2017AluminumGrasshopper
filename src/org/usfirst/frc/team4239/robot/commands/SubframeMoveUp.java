package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SubframeMoveUp extends Command {

    public SubframeMoveUp() {
    	requires(Robot.subframe);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.subframe.moveUp();
    }

    protected boolean isFinished() {
        return Robot.subframe.getTopLimitSwitch();
    }

    protected void end() {
    	Robot.subframe.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
