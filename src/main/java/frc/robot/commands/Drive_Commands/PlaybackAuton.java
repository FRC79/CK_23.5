// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotRecorder;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.DriveConstants;;

public class PlaybackAuton extends CommandBase {

  private RobotRecorder m_Recorder;
  private DriveTrain m_DriveTrain;

  // values retrieved from robotRecorder
  private Double joyX;
  private Double joyY;
  private Double leftEncoderDistRecorded;
  private Double rightEncoderDistRecorded;
  private double proportionWeight;
  private double derivativeWeight;
  private double PIDWeight;
  

  /** Creates a new PlaybackAuton. */
  public PlaybackAuton(RobotRecorder subsystem, DriveTrain dTrain) {
    // Use addRequirements() here to declare subsystem dependencies
    m_Recorder = subsystem;
    m_DriveTrain = dTrain;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Recorder.startPlayback();
    m_DriveTrain.resetEncoders();
    PIDWeight = DriveConstants.PID_WEIGHT;
    proportionWeight = DriveConstants.PROPORTION_WEIGHT;
    derivativeWeight = DriveConstants.DERIVATIVE_WEIGHT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Recorder.getRobotData("joyX") == null 
    || m_Recorder.getRobotData("joyY") == null 
    || m_Recorder.getRobotData("leftEncoderDist") == null
    || m_Recorder.getRobotData("leftEncoderDist") == null
    ){
      return;
    }
    // get values from the recording
    joyX = m_Recorder.getRobotData("joyX");
    joyY = -1 *m_Recorder.getRobotData("joyY");
    leftEncoderDistRecorded = m_Recorder.getRobotData("leftEncoderDist");
    rightEncoderDistRecorded = m_Recorder.getRobotData("rightEncoderDist");

    // proportion = (what the distance should be - what the measured distance is) * weight
    double leftProp = (leftEncoderDistRecorded - m_DriveTrain.leftEncoderDistance())*proportionWeight;
    double rightProp = (rightEncoderDistRecorded - m_DriveTrain.rightEncoderDistance())*proportionWeight;

    // derivative = proportion' aka velocity * weight
    double leftDeriv = (m_DriveTrain.leftEncoderVelocity())*derivativeWeight;
    double rightDeriv = (m_DriveTrain.rightEncoderVelocity())*derivativeWeight;

    /*
    // convert arcade to left and right and add then correction throttle
    double left  = (joyY + joyX) + (leftProp - leftDeriv)*PIDWeight;
    double right = (joyY - joyX) + (rightProp - rightDeriv)*PIDWeight;	
    */

    // make left and right simply the PID correction to try and get the drive train to the correct distances
    double left = (leftProp - leftDeriv)*PIDWeight;
    double right = (rightProp - rightDeriv)*PIDWeight;
    // drive train 
    m_DriveTrain.tankDrive(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: make this command end if safety values don't line up or something
    return (m_Recorder.getRobotData("joyX") == null 
    || m_Recorder.getRobotData("joyY") == null 
    || m_Recorder.getRobotData("leftEncoderDist") == null 
    || m_Recorder.getRobotData("rightEncoderDist") == null );
  }
}
