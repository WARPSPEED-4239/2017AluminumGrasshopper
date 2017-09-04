package org.usfirst.frc.team4239.robot;

import org.usfirst.frc.team4239.robot.commands.AgitatorsReverse;
import org.usfirst.frc.team4239.robot.commands.AgitatorsSpin;
import org.usfirst.frc.team4239.robot.commands.ClimberClimb;
import org.usfirst.frc.team4239.robot.commands.DrivetrainArcadeDriveGyroAssist;
import org.usfirst.frc.team4239.robot.commands.DrivetrainDriveToDistance;
import org.usfirst.frc.team4239.robot.commands.DrivetrainRotateToAngle;
import org.usfirst.frc.team4239.robot.commands.GearPickupClampOut;
import org.usfirst.frc.team4239.robot.commands.GearPickupLiftDown;
import org.usfirst.frc.team4239.robot.commands.GearPickupLiftUp;
import org.usfirst.frc.team4239.robot.commands.ShooterSpin;
import org.usfirst.frc.team4239.robot.commands.SubframeMoveDown;
import org.usfirst.frc.team4239.robot.commands.SubframeMoveUp;
import org.usfirst.frc.team4239.robot.commands.autonomous.AutonDriveBackwardNoSensors;
import org.usfirst.frc.team4239.robot.commands.autonomous.AutonDriveForward;
import org.usfirst.frc.team4239.robot.commands.autonomous.AutonDriveForwardNoSensors;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
	private XboxController controller;
	private Joystick joystick;
	
	public OI() {
		controller = new XboxController(0);
		joystick = new Joystick(1);
		
		addXboxControllerControls();
		addJoystickControls();
		
		
		
		if (RobotMap.DEBUG) {
			addSmartDashboardControls();
		}
		

	}	
	
	private void addXboxControllerControls() {
		// Note: A_BUTTON is used to slow down Drivetrain
		
		final int B_BUTTON_INDEX = 2,
				  X_BUTTON_INDEX = 3,
				  Y_BUTTON_INDEX = 4;
				  /*LEFT_BUMPER_INDEX = 5,
				  RIGHT_BUMPER_INDEX = 6;*/
		
		final JoystickButton B_BUTTON = new JoystickButton(controller, B_BUTTON_INDEX),
							 X_BUTTON = new JoystickButton(controller, X_BUTTON_INDEX),
					         Y_BUTTON = new JoystickButton(controller, Y_BUTTON_INDEX);
					         /*LEFT_BUMPER = new JoystickButton(controller, LEFT_BUMPER_INDEX),
					         RIGHT_BUMPER = new JoystickButton(controller, RIGHT_BUMPER_INDEX);*/
		
		/*LEFT_BUMPER.whenPressed(new GearPickupMechanismMovePiston(PickupPosition.ClampOut));
		RIGHT_BUMPER.whenPressed(new GearPickupMechanismMovePiston(PickupPosition.ClampIn));*/
		
		B_BUTTON.toggleWhenPressed(new GearPickupClampOut());
		X_BUTTON.toggleWhenPressed(new DrivetrainArcadeDriveGyroAssist());
		Y_BUTTON.whileHeld(new ClimberClimb());
	}
	
	private void addJoystickControls() {
		final JoystickButton BUTTON_1 = new JoystickButton(joystick, 1),
				             BUTTON_3 = new JoystickButton(joystick, 3),
				             BUTTON_4 = new JoystickButton(joystick, 4),
				             BUTTON_5 = new JoystickButton(joystick, 5),
				             BUTTON_6 = new JoystickButton(joystick, 6),
				             BUTTON_11 = new JoystickButton(joystick, 11),
				             BUTTON_12 = new JoystickButton(joystick, 12);
		
		BUTTON_1.whileHeld(new ShooterSpin());
		BUTTON_3.whenPressed(new GearPickupLiftDown());
		BUTTON_4.whenPressed(new GearPickupLiftUp());
		BUTTON_5.whileHeld(new AgitatorsReverse());
		BUTTON_6.whileHeld(new AgitatorsSpin());
		BUTTON_11.whileHeld(new SubframeMoveDown());
		BUTTON_12.whileHeld(new SubframeMoveUp());
	}
	
	
	private void addSmartDashboardControls() {
		SmartDashboard.putData("Drive Forward 1ft", new DrivetrainDriveToDistance(1.0));
		SmartDashboard.putData("Drive Forward 5ft", new DrivetrainDriveToDistance(5.0));
		SmartDashboard.putData("Drive Backward 1ft", new DrivetrainDriveToDistance(-1.0));
		SmartDashboard.putData("Drive Backward 5ft", new DrivetrainDriveToDistance(-5.0));
		SmartDashboard.putData("Rotate Left 5 degrees", new DrivetrainRotateToAngle(-5.0));
		SmartDashboard.putData("Rotate Left 90 degrees", new DrivetrainRotateToAngle(-90.0));
		SmartDashboard.putData("Rotate Right 5 degrees", new DrivetrainRotateToAngle(5.0));
		SmartDashboard.putData("Rotate Right 90 degrees", new DrivetrainRotateToAngle(90.0));
		
		SmartDashboard.putData("AutonDriveForward", new AutonDriveForward());
		SmartDashboard.putData("AutonDriveForward (no sensors)", new AutonDriveForwardNoSensors());
		SmartDashboard.putData("AutonDriveBackward (no sensors)", new AutonDriveBackwardNoSensors());
		
		/*
		SmartDashboard.putData("FuelIntake Stop", new FuelIntakeStop());
		SmartDashboard.putData("Fuel Intake Reverse", new FuelIntakeReverse());
		SmartDashboard.putData("Fuel Intake Pickup", new FuelIntakePickup());
		
		SmartDashboard.putData("Agitators Spin", new AgitatorsSpin());
		SmartDashboard.putData("Agitators Stop", new AgitatorsStop());
		SmartDashboard.putData("Agitators Reverse", new AgitatorsReverse());
		
		SmartDashboard.putData("Climber Climb", new ClimberClimb());
		SmartDashboard.putData("Climber Stop", new ClimberStop());
		
		SmartDashboard.putData("GearPickup LiftInClampIn", 
				               new GearPickupMechanismMovePiston(PickupPosition.LiftInClampIn));
		SmartDashboard.putData("GearPickup LiftInClampOut",
				               new GearPickupMechanismMovePiston(PickupPosition.LiftInClampOut));
		SmartDashboard.putData("GearPickup LiftOutClampIn",
				               new GearPickupMechanismMovePiston(PickupPosition.LiftOutClampIn));
		SmartDashboard.putData("GearPickup LiftOutClampOut",
				               new GearPickupMechanismMovePiston(PickupPosition.LiftOutClampOut));
		
		SmartDashboard.putData("GearRearMechanism PistonIn",
				               new GearRearManipulatorMovePiston(RearPistonPosition.PistonIn));
		SmartDashboard.putData("GearRearMechanism PistonOut",
	                           new GearRearManipulatorMovePiston(RearPistonPosition.PistonOut));
		20
		SmartDashboard.putData("Shooter Spin", new ShooterSpin(0.5));
		SmartDashboard.putData("Shooter Stop", new ShooterStop());
		
		SmartDashboard.putData("Subframe Move Up", new SubframeMoveUp());
		SmartDashboard.putData("Subframe Move Down", new SubframeMoveDown());
		SmartDashboard.putData("Subframe Stop", new SubframeStop());
		*/
		
	}
	
	
	public XboxController getController() {
		return controller;
	}
	
	public Joystick getJoystick() {
		return joystick;
	}
	
}
