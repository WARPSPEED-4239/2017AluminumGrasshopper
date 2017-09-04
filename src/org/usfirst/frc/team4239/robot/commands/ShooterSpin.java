package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSpin extends Command {

	private boolean isPreset;
	private double presetSpeed;
	
	public ShooterSpin(double speed) {
		requires(Robot.shooter);
		presetSpeed = speed;
		isPreset = true;
	}
	
    public ShooterSpin() {
        requires(Robot.shooter);
        isPreset = false;
    }

    protected void initialize() {}

    protected void execute() {
    	double speed = 0.0;
    	
    	if (isPreset) {
    		speed = presetSpeed;
    	}
    	else {
    		double speedFromJoystick = Robot.oi.getJoystick().getThrottle(); // 1 to -1
    		speedFromJoystick *= -1;                                         // -1 to 1
    		speedFromJoystick += 1;                                          // 0 to 2
    		speedFromJoystick *= 0.35;                                       // 0 to 0.7
    		speedFromJoystick += 0.70;                                       // 0.3 to 1
    		speedFromJoystick -= 0.40;
    		speed = speedFromJoystick;
    	}
    	
    	SmartDashboard.putNumber("Shooter Speed", speed);
    	Robot.shooter.spinAtSpeed(speed);
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    	Robot.shooter.stop();
    }

    protected void interrupted() {
    	end();
    }
    
}
