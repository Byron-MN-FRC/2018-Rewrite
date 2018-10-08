package org.usfirst.frc.team4859.robot.subsystems;

import org.usfirst.frc.team4859.robot.RobotMap;
import org.usfirst.frc.team4859.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team4859.robot.commands.DriveWithJoystickAssisted;

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
	static final double kp = 0.03f;
	static final double ki = 0;
	static final double kd = 0;
	static final double kperiod = 1000f;
	static final double kToleranceDegrees = 2.0f;    

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
	private double pidOutput = 0; // based on angle of robot.
	
	private boolean operatorAssist = false;  // When driving straight, operatorAssist will keep you straight.
	
	
    // Initialize your subsystem here
    public PIDDriveTrain() {
    	super("PIDDriveTrain", kp, ki, kd, 0);
    	
        try {
        	NAVX_ahrs = new AHRS(SerialPort.Port.kUSB1); 
        	NAVX_ahrs.reset();
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        setInputRange(-180.0f,  180.0f);
        setOutputRange(-1.0, 1.0);
        setAbsoluteTolerance(kToleranceDegrees);
        this.getPosition();
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return NAVX_ahrs.getYaw();
    }

    @Override
    protected void usePIDOutput(double output) {
    	pidOutput = output;
    	//System.out.println(pidOutput);
    }
    
    
	public void initDefaultCommand () {
		setDefaultCommand(new DriveWithJoystickAssisted());
	}

	public void driveWithJoystick(Joystick joyStick) {
		// store the current Y and Twist values in local fields for updateStatus
		joystickY     = joyStick.getY();
		joystickTwist = joyStick.getTwist();
		
		// If we are driving straight (no twist) use navx && PID to keep us straight
		if (Math.abs(joystickTwist) < 0.2) {
			if (!operatorAssist) {
				//NAVX_ahrs.reset();
				NAVX_ahrs.zeroYaw();
				operatorAssist = true;
				setSetpoint(NAVX_ahrs.getYaw());
				pidOutput = 0; // initialize it, will be changed by PID system
				enable();
			}
		}
		else {operatorAssist = false; disable();}
	
		// The rotation portion of arcadeDrive will use pidOutput if operatorAssist is true, 
		// otherwise, will use the joystick twist.
		drivetrain.arcadeDrive(-joystickY, (operatorAssist ? pidOutput : joystickTwist));
	}
	
	public void updateStatus() {
		SmartDashboard.putNumber("Joystick-Y", joystickY);
		SmartDashboard.putNumber("Joystick-Twist", joystickTwist);
		SmartDashboard.putNumber("PID Correction", pidOutput);
		SmartDashboard.putBoolean("DriveStraightAssist", operatorAssist);
		SmartDashboard.putNumber("Navx Yaw", NAVX_ahrs.getYaw());
		SmartDashboard.putNumber("NavX Pitch", NAVX_ahrs.getPitch());
		SmartDashboard.putNumber("Navx Roll", NAVX_ahrs.getRoll());
		SmartDashboard.putData(this.getPIDController());
	}
	
	public void stop() {
		drivetrain.arcadeDrive(0, 0);
	}
    
}
