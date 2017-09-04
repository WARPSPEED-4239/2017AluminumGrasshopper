package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class FuelIntakeStop extends Command {

    public FuelIntakeStop() {
        requires(Robot.fuelIntake);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.fuelIntake.stop();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {}

    protected void interrupted() {}
    
}
