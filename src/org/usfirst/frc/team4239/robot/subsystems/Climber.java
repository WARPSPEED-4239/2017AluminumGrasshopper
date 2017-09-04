package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.ClimberStop;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Climber extends Subsystem {

    private SpeedController climberMotor;
    
    public Climber() {
    	super();
    	
    	climberMotor = new Spark(RobotMap.MOTOR_CLIMBER);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ClimberStop());
    }
    
    public void stop() {
    	climberMotor.stopMotor();
    }
    
    public void climb() {
    	climberMotor.set(1.0);
    }
    
    public void reverse() {
    	climberMotor.set(-1.0);
    }
    
}

