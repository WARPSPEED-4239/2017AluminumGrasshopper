package org.usfirst.frc.team4239.robot.commands;

import org.usfirst.frc.team4239.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import motion.control.MotionController;
import motion.profiling.MotionProfile;
import motion.profiling.MotionProfiler;

/**
 *
 */
public class DrivetrainFollowDistanceProfile extends Command {

	private final double MAX_VELOCITY = 10;
	private final double MAX_ACCELERATION = 15;
	
	private final double CRUISE_VELOCITY = 4.0;
	private final double ACCELERATION_RATE = 0.5;
	
	private MotionController motionController;
	private MotionProfiler motionProfiler = new MotionProfiler();
	
    public DrivetrainFollowDistanceProfile(double distance) {
    	requires(Robot.drivetrain);
    	
    	double Kp = 0.0;   //1*
    	double Ki = 0.0;   //2
    	double Kd = 0.0;   //3
    	double Kv = 1 / MAX_VELOCITY;
    	double Ka = 1 / MAX_ACCELERATION;
    	
    	motionController = new MotionController(
    			Robot.drivetrain.getDistanceSource(), 
    			Robot.drivetrain.getDistanceOutput(), 
    			Kp, Ki, Kd, Kv, Ka
    	);
    	
    	MotionProfile motionProfile = motionProfiler.getProfile(distance, CRUISE_VELOCITY, ACCELERATION_RATE);
    	motionController.setMotionProfile(motionProfile);
    	motionController.setTolerance(1/12);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.resetSensors();
    	
    	motionController.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return motionController.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	motionController.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
