package org.usfirst.frc.team4239.robot.commands.autonomous;

import org.usfirst.frc.team4239.robot.commands.DrivetrainDriveToDistance;
import org.usfirst.frc.team4239.robot.commands.GearPickupClampIn;
import org.usfirst.frc.team4239.robot.commands.GearPickupClampOut;
import org.usfirst.frc.team4239.robot.commands.GearPickupLiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutonDriveForwardDeployGear extends CommandGroup {

    public AutonDriveForwardDeployGear() {
    	addParallel(new GearPickupLiftUp());
    	addParallel(new GearPickupClampIn());
        addSequential(new DrivetrainDriveToDistance(6.55), 4.0);
        addSequential(new WaitCommand(0.5));
        addParallel(new GearPickupClampOut());
        addSequential(new DrivetrainDriveToDistance(-4.55), 2.0);
    }
    
}
