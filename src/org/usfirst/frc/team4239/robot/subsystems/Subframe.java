package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.SubframeStop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Subframe extends Subsystem {

    private SpeedController subframeMotor;
    private DigitalInput topLimitSwitch, bottomLimitSwitch;
    
    public Subframe() {
    	super();
    	
    	subframeMotor = new Spark(RobotMap.MOTOR_SUBFRAME);
    	
    	topLimitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_SUBFRAME_TOP);
    	bottomLimitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_SUBFRAME_BOTTOM);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new SubframeStop());
    }
    
    public void stop() {
    	updateSmartDashboard();
    	subframeMotor.stopMotor();
    }
    
    public void moveUp() {
    	updateSmartDashboard();
    	subframeMotor.set(1.0);
    }
    
    public void moveDown() {
    	updateSmartDashboard();
    	subframeMotor.set(-1.0);
    }
    
    public boolean getTopLimitSwitch() {
    	return !topLimitSwitch.get();
    }
    
    public boolean getBottomLimitSwitch() {
    	return !bottomLimitSwitch.get();
    }
    
    private void updateSmartDashboard() {
    	SmartDashboard.putBoolean("Top Limit Switch", getTopLimitSwitch());
    	SmartDashboard.putBoolean("Bottom Limit Switch", getBottomLimitSwitch());
    }
    
}

