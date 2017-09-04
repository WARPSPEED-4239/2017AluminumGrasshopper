package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.GearPickupClampIn;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearPickupClamp extends Subsystem {

	private DoubleSolenoid clampSolenoid = new DoubleSolenoid(RobotMap.SOLENOID_GEAR_PICKUP_MECHANISM_CLAMP_FORWARD, RobotMap.SOLENOID_GEAR_PICKUP_MECHANSIM_CLAMP_REVERSE);
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new GearPickupClampIn());
    }
    
    public void stop () {
    	clampSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    public void open () {
    	clampSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void close () {
    	clampSolenoid.set(DoubleSolenoid.Value.kReverse);    
    }
}

