package org.usfirst.frc.team4239.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DrivetrainSpeedTest extends Command {

	class PositionData {
		public double time, position, velocity;
	}
	
	private ArrayList<PositionData> distanceList = new ArrayList<PositionData>();
	private double startTime, currentTime;
	
    public DrivetrainSpeedTest() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(10.0);
    	startTime = Timer.getFPGATimestamp();
    	distanceList.clear();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.arcadeDrive(1.0, 0.0);
    	
    	currentTime = Timer.getFPGATimestamp();
    	
    	PositionData data = new PositionData();
    	data.time = currentTime - startTime;
    	data.position = Robot.drivetrain.getDistance();
    	data.velocity = Robot.drivetrain.getRate();
    	distanceList.add(data);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("time, position, velocity");
    	for (PositionData data : distanceList)
    		System.out.println(data.time + ", " + data.position + ", " + data.velocity);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
