package org.usfirst.frc.team4859.robot.subsystems;

import org.usfirst.frc.team4859.robot.RobotMap;
import org.usfirst.frc.team4859.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	public static WPI_TalonSRX motorLeftMaster = new WPI_TalonSRX(RobotMap.talonIDLeftMaster);
	public static WPI_TalonSRX motorLeftFollower = new WPI_TalonSRX(RobotMap.talonIDLeftFollower);
	
	public static WPI_TalonSRX motorRightMaster = new WPI_TalonSRX(RobotMap.talonIDRightMaster);
	public static WPI_TalonSRX motorRightFollower = new WPI_TalonSRX(RobotMap.talonIDRightFollower);
	
	public static SpeedControllerGroup drivetrainLeft = new SpeedControllerGroup(motorLeftMaster);
	public static SpeedControllerGroup drivetrainRight = new SpeedControllerGroup(motorRightMaster);
	
	public static DifferentialDrive drivetrain = new DifferentialDrive(drivetrainLeft, drivetrainRight);
	
	private double joystickY;
	private double joystickTwist;

	public void initDefaultCommand () {
		setDefaultCommand(new DriveWithJoystick());
	}

	public void driveWithJoystick(Joystick joyStick) {
		// store the current Y and Twist values in local fields for updateStatus
		joystickY     = joyStick.getY();
		joystickTwist = joyStick.getTwist();
		
		drivetrain.arcadeDrive(joystickY, joystickTwist);
	}
	
	public void updateStatus() {
		SmartDashboard.putNumber("Joystick-Y", joystickY);
		SmartDashboard.putNumber("Joystick-Twist", joystickY);
	}
	
	public void stop() {
		drivetrain.arcadeDrive(0, 0);
	}
}

