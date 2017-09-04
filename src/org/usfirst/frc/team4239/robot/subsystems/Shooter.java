package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.ShooterStop;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {

    private SpeedController shooterMotor;
	
	public Shooter() {
		super();
		
		shooterMotor = new Talon(RobotMap.MOTOR_SHOOTER);
		shooterMotor.setInverted(true);
	}

    public void initDefaultCommand() {
        setDefaultCommand(new ShooterStop());
    }
    
    public void stop() {
    	shooterMotor.stopMotor();
    }
    
    public void spinAtSpeed(double speed) {
    	shooterMotor.set(speed);
    }
}

