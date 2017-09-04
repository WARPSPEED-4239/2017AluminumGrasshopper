package org.usfirst.frc.team4239.robot.commands.autonomous;

import org.usfirst.frc.team4239.robot.commands.DrivetrainDriveToDistance;
import org.usfirst.frc.team4239.robot.commands.GearPickupClampIn;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonDriveForward extends CommandGroup {

    public AutonDriveForward() {
        addParallel(new GearPickupClampIn());
        addSequential(new DrivetrainDriveToDistance(6.0), 4.0);
    }
    
}