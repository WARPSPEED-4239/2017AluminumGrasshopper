package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class FuelIntakeReverse extends Command {

    public FuelIntakeReverse() {
    	requires(Robot.fuelIntake);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.fuelIntake.reverse();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    	Robot.fuelIntake.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
