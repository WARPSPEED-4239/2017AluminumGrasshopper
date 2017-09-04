package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberStop extends Command {

    public ClimberStop() {
        requires(Robot.climber);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.climber.stop();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {}

    protected void interrupted() {}
   
}
