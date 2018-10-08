package org.usfirst.frc.team4859.robot.subsystems;

import org.usfirst.frc.team4859.robot.RobotMap;
import org.usfirst.frc.team4859.robot.commands.DriveWithJoystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class PIDDriveTrain extends PIDSubsystem {
	static final double kp = 0;
	static final double ki = 0;
	static final double kd = 0;
	static final double kPeriod= 1f;
	static final double kToleranceDegrees = 0.5f;    

	public static WPI_TalonSRX motorLeftMaster = new WPI_TalonSRX(RobotMap.talonIDLeftMaster);
	public static WPI_TalonSRX motorLeftFollower = new WPI_TalonSRX(RobotMap.talonIDLeftFollower);
	
	public static WPI_TalonSRX motorRightMaster = new WPI_TalonSRX(RobotMap.talonIDRightMaster);
	public static WPI_TalonSRX motorRightFollower = new WPI_TalonSRX(RobotMap.talonIDRightFollower);
	
	public static SpeedControllerGroup drivetrainLeft = new SpeedControllerGroup(motorLeftMaster);
	public static SpeedControllerGroup drivetrainRight = new SpeedControllerGroup(motorRightMaster);
	
	public static DifferentialDrive drivetrain = new DifferentialDrive(drivetrainLeft, drivetrainRight);
	
	private AHRS NAVX_ahrs; // Attitude Heading Reference System
	private double joystickY;
	private double joystickTwist;
	private double pidOutput = 0.0; // based on angle of robot.
	
	private boolean operatorAssist = false;  // When driving straight, operatorAssist will keep you straight.
	
	
    // Initialize your subsystem here
    public PIDDriveTrain() {
    	super("PIDDriveTrain", kp, ki, kd, 0, kPeriod);
    	
        try {
        	NAVX_ahrs = new AHRS(SerialPort.Port.kUSB1); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        setInputRange(-180.0f,  180.0f);
        setOutputRange(-1.0, 1.0);
        setAbsoluteTolerance(kToleranceDegrees);
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return NAVX_ahrs.getYaw();
    }

    protected void usePIDOutput(double output) {
    	pidOutput = output;
    }
    
	public void initDefaultCommand () {
		setDefaultCommand(new DriveWithJoystick());
	}

	public void driveWithJoystick(Joystick joyStick) {
		// store the current Y and Twist values in local fields for updateStatus
		joystickY     = joyStick.getY();
		joystickTwist = joyStick.getTwist();
		
		// If we are driving straight (no twist) use navx && PID to keep us straight
		if ((joystickTwist == 0) && (joystickY != 0)){
			if (!operatorAssist) {
				operatorAssist = true;
				setSetpoint(NAVX_ahrs.getYaw());
				pidOutput = 0; // initialize it, will be changed by PID system
				enable();
			}
		}
		else {operatorAssist = false; disable();}
		
		// The rotation portion of arcadeDrive will use pidOutput if operatorAssist is true, 
		// otherwise, will use the joystick twist.
		drivetrain.arcadeDrive(joystickY, (operatorAssist ? pidOutput : joystickTwist));
	}
	
	public void updateStatus() {
		SmartDashboard.putNumber("Joystick-Y", joystickY);
		SmartDashboard.putNumber("Joystick-Twist", joystickY);
		SmartDashboard.putNumber("PID Correction", pidOutput);
	}
	
	public void stop() {
		drivetrain.arcadeDrive(0, 0);
	}
    
}
