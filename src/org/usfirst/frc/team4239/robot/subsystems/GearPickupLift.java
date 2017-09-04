package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.GearPickupLiftDown;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearPickupLift extends Subsystem {
	
	private DoubleSolenoid liftSolenoid = new DoubleSolenoid(RobotMap.SOLENOID_GEAR_PICKUP_MECHANISM_LIFT_FORWARD, RobotMap.SOLENOID_GEAR_PICKUP_MECHANISM_LIFT_REVERSE);

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new GearPickupLiftDown());
       
    }
    
    public void stop () {
        liftSolenoid.set(DoubleSolenoid.Value.kOff);
    }
        
    public void down () {
        liftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
        
    public void up () {
        liftSolenoid.set(DoubleSolenoid.Value.kForward);    
    }
}