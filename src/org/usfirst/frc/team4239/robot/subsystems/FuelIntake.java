package org.usfirst.frc.team4239.robot.subsystems;

import org.usfirst.frc.team4239.robot.RobotMap;
import org.usfirst.frc.team4239.robot.commands.FuelIntakeStop;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

public class FuelIntake extends Subsystem {

    private SpeedController fuelIntakeMotor;
    
    public FuelIntake() {
    	super();
    	
    	fuelIntakeMotor = new Talon(RobotMap.MOTOR_FUEL_INTAKE);
    	fuelIntakeMotor.setInverted(true);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new FuelIntakeStop());
    }
    
    public void stop() {
    	fuelIntakeMotor.stopMotor();
    }
    
    public void pickup() {
    	fuelIntakeMotor.set(1.0);
    }
    
    public void reverse() {
    	fuelIntakeMotor.set(-1.0);
    }
    
}

