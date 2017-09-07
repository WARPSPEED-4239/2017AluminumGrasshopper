package org.usfirst.frc.team4239.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4239.robot.Robot;

public class DrivetrainArcadeDrive extends Command {
	
	public DrivetrainArcadeDrive() {
		requires(Robot.drivetrain);
	}

	@Override
	protected void initialize() {}

	private double maxVelocity = 0.0;
	private double velocity;
	
	@Override
	protected void execute() {
		Robot.drivetrain.arcadeDrive(Robot.oi.getController());
		
		velocity = Robot.drivetrain.getLinearVelocity();
		if (Math.abs(velocity) > maxVelocity) {
			maxVelocity = Math.abs(velocity);
		}
		
		SmartDashboard.putNumber("velocity", velocity);
		SmartDashboard.putNumber("maxVelocity", maxVelocity);
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
