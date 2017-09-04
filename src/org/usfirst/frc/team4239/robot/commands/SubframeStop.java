package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SubframeStop extends Command {

    public SubframeStop() {
        requires(Robot.subframe);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.subframe.stop();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {}

    protected void interrupted() {}
    
}
