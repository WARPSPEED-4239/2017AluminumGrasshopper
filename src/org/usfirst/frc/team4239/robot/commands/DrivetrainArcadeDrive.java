package org.usfirst.frc.team4239.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team4239.robot.Robot;

public class DrivetrainArcadeDrive extends Command {
	
	public DrivetrainArcadeDrive() {
		requires(Robot.drivetrain);
	}

	@Override
	protected void initialize() {}

	@Override
	protected void execute() {
		Robot.drivetrain.arcadeDrive(Robot.oi.getController());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {}

	@Override
	protected void interrupted() {}
	
}
