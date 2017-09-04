
package org.usfirst.frc.team4239.robot;

import org.usfirst.frc.team4239.robot.commands.autonomous.AutonDriveBackwardNoSensors;
import org.usfirst.frc.team4239.robot.commands.autonomous.AutonDriveForward;
import org.usfirst.frc.team4239.robot.commands.autonomous.AutonDriveForwardNoSensors;
import org.usfirst.frc.team4239.robot.subsystems.Agitators;
import org.usfirst.frc.team4239.robot.subsystems.Climber;
import org.usfirst.frc.team4239.robot.subsystems.Drivetrain;
import org.usfirst.frc.team4239.robot.subsystems.FuelIntake;
import org.usfirst.frc.team4239.robot.subsystems.GearPickupClamp;
import org.usfirst.frc.team4239.robot.subsystems.GearPickupLift;
import org.usfirst.frc.team4239.robot.subsystems.Shooter;
import org.usfirst.frc.team4239.robot.subsystems.Subframe;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final Agitators agitators = new Agitators();
	public static final Climber climber = new Climber();
	public static final Drivetrain drivetrain = new Drivetrain();
	public static final FuelIntake fuelIntake = new FuelIntake();
	public static final GearPickupLift gearPickupLift = new GearPickupLift();
	public static final GearPickupClamp gearPickupClamp = new GearPickupClamp();
	public static final Shooter shooter = new Shooter();
	public static final Subframe subframe = new Subframe();
	
	public static OI oi;

	private Command autonomousCommand;
	private SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		
		
		UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
		cam0.setResolution(320, 240);
		cam0.setFPS(10);
		
		UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(1);
		cam1.setResolution(320, 240);
		cam1.setFPS(10);
		
		
		chooser.addDefault("Drive Forward No Sensors", new AutonDriveForwardNoSensors());
		chooser.addObject("Drive Forward", new AutonDriveForward());
		chooser.addObject("Drive Backward No Sensors", new AutonDriveBackwardNoSensors());
		SmartDashboard.putData("Auto Mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		updateSmartDashboard();
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
		drivetrain.resetSensors();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		updateSmartDashboard();
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	public void updateSmartDashboard() {
		SmartDashboard.putData("Agitators", agitators);
		SmartDashboard.putData("Climber", climber);
		SmartDashboard.putData("Drivetrain", drivetrain);
		SmartDashboard.putData("Fuel Intake", fuelIntake);
		SmartDashboard.putData("Shooter", shooter);
		SmartDashboard.putData("Subframe", subframe);
		SmartDashboard.putData("Gear Lift", gearPickupLift);
		SmartDashboard.putData("Gear Clamp", gearPickupClamp);
	}
}
