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
  private final Encoder rightEncoder = new Encoder(DriveConstants.LEFT_ENCODER_PORT_A,DriveConstants.LEFT_ENCODER_PORT_B,DriveConstants.LEFT_ENCODER_REVERSE);
  private final Encoder leftEncoder  = new Encoder(DriveConstants.LEFT_ENCODER_PORT_A,DriveConstants.LEFT_ENCODER_PORT_B,DriveConstants.LEFT_ENCODER_REVERSE);

  /* gyro */
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); ;

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
  public DriveTrain() {}

  // sets motors based on forward and turn inputs 
  public void arcadeDrive(double fwd, double rot) {

    /* eliminate imperfection in restin position of joystick */
		double forward = Deadband(fwd);
    double turn = Deadband(rot);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
    frontLeftMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
    backLeftMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
    
    frontRightMotor.set(ControlMode.PercentOutput, -forward, DemandType.ArbitraryFeedForward, turn);
    backRightMotor.set(ControlMode.PercentOutput, -forward, DemandType.ArbitraryFeedForward, turn);
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
    vel1 = rightEncoder.getRate();
    vel2 = leftEncoder.getRate();

    m_dash.putNumber("Encoder1", vel1);
    m_dash.putNumber("Encoder2", vel2);

    angle = gyro.getAngle();
    altitude = gyro.getAltitude();
    heading = gyro.getCompassHeading();

    m_dash.putNumber("Angle", angle);
    m_dash.putNumber("Altitude", altitude);
    m_dash.putNumber("Altitude", heading);

  }
}
