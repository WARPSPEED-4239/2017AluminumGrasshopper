package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class AgitatorsStop extends Command {

    public AgitatorsStop() {
        requires(Robot.agitators);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.agitators.stop();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {}

    protected void interrupted() {}
    
}
