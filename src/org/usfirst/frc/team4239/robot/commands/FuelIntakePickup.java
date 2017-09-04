package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class FuelIntakePickup extends Command {

    public FuelIntakePickup() {
    	requires(Robot.fuelIntake);
    }

    protected void initialize() {}

    protected void execute() {
    	Robot.fuelIntake.pickup();
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
