package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SubframeMoveDown extends Command {

    public SubframeMoveDown() {
        requires(Robot.subframe);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.subframe.moveDown();
    }

    protected boolean isFinished() {
        return Robot.subframe.getBottomLimitSwitch();
    }

    protected void end() {
    	Robot.subframe.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
