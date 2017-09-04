package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.AgitatorsStop;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Agitators extends Subsystem {

    private SpeedController Agitator;

    public Agitators() {
    	super();
    	
    	Agitator = new Talon(RobotMap.MOTOR_HOPPER_AGITATOR);
    	Agitator.setInverted(true);
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(new AgitatorsStop());
    }
    
    public void stop() {
    	Agitator.stopMotor();
    }
    
    public void spin() {
    	Agitator.set(.4);
    }
    
    public void reverse() {
    	Agitator.set(-.4);
    }
    
}

