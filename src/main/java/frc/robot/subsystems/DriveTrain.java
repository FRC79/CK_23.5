// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import constants
import frc.robot.Constants.DriveConstants;

// encoder lib
import edu.wpi.first.wpilibj.Encoder;

// gyro lib
import com.kauailabs.navx.frc.AHRS;
// SP interface for gyro
import edu.wpi.first.wpilibj.SPI;

// dashboard for debug values
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// libraries for victors and talons
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class DriveTrain extends SubsystemBase {

  /* left motors */
  private final VictorSPX frontLeftMotor = new VictorSPX(DriveConstants.LEFT_MOTOR1_PORT);
  private final TalonSRX backLeftMotor = new TalonSRX(DriveConstants.LEFT_MOTOR2_PORT);

  /* right motors */
  private final TalonSRX frontRightMotor = new TalonSRX(DriveConstants.RIGHT_MOTOR1_PORT);
  private final VictorSPX backRightMotor = new VictorSPX(DriveConstants.RIGHT_MOTOR2_PORT);

  /* encoders */
  private final Encoder leftEncoder = new Encoder(DriveConstants.LEFT_ENCODER_PORT_A,DriveConstants.LEFT_ENCODER_PORT_B,DriveConstants.LEFT_ENCODER_REVERSE);
  private final Encoder rightEncoder  = new Encoder(DriveConstants.RIGHT_ENCODER_PORT_A,DriveConstants.RIGHT_ENCODER_PORT_B,DriveConstants.RIGHT_ENCODER_REVERSE);

  /* gyro */
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); ;

  /* encoder constants */
  private final double diameter = 6; // inches
  private final double cpr = 649.21875; // counts per rotation (gear rotation included)


  /* dashboard for debug values */
  private SmartDashboard m_dash;


  /* values for testing encoders */
  double vel1;
  double vel2;

  /* values for testing gyro */
  double angle;
  double altitude;
  double heading;

  /** Creates a new Drivetrain. */
  public DriveTrain() {
    rightEncoder.setDistancePerPulse((18d/4.808701d)*Math.PI*diameter/cpr);
    leftEncoder.setDistancePerPulse((18d/4.808701d)*Math.PI*diameter/cpr);
    resetGyro();
  }

  // gyro reset
  public void resetGyro() {
    gyro.reset();
  }

  public double gyroRate() {
    return gyro.getRate();
  }

  // gyro get
  public double getHeading(){
    return gyro.getAngle();
  }

  // set the distance of the encoders to 0
  public void resetEncoders() {
    rightEncoder.reset();
    leftEncoder.reset();
  }

  // get distance and velocity from the encoders
  public Double rightEncoderDistance() {
    return rightEncoder.getDistance();
  }
  public Double leftEncoderDistance() {
    return leftEncoder.getDistance();
  }
  public Double rightEncoderVelocity() {
    return rightEncoder.getRate();
  }
  public Double leftEncoderVelocity() {
    return leftEncoder.getRate();
  }

  // sets motors based on forward and turn inputs 
  public void arcadeDrive(double fwd, double rot) {

    /* eliminate imperfection in resting position of joystick */
		double forward = Deadband(fwd);
    double turn = Deadband(rot);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
    frontLeftMotor.set(ControlMode.PercentOutput, -forward, DemandType.ArbitraryFeedForward, -turn);
    backLeftMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
    
    frontRightMotor.set(ControlMode.PercentOutput, -forward, DemandType.ArbitraryFeedForward, turn);
    backRightMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
  }

  // sets motors based on throttle per side of drive train
  public void tankDrive(double left, double right) {

    /* eliminate imperfection in resting position of joystick */
    double leftThrottle = Deadband(left);
    double rightThrottle = Deadband(right);

    /* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
    frontLeftMotor.set(ControlMode.PercentOutput, -leftThrottle);
    backLeftMotor.set(ControlMode.PercentOutput, leftThrottle);
    
    frontRightMotor.set(ControlMode.PercentOutput, -rightThrottle);
    backRightMotor.set(ControlMode.PercentOutput, rightThrottle);
  }

  // eliminates small inperfections in the driveStick's resting position
  double Deadband(double value) { 
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    vel1 = rightEncoder.getDistance();
    vel2 = leftEncoder.getDistance();

    m_dash.putNumber("right Encoder", vel1);
    m_dash.putNumber("left Encoder", vel2);

    m_dash.putNumber("angle", gyro.getAngle());

  }
}
