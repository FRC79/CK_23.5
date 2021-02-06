// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotRecorder;
import frc.robot.subsystems.DriveTrain;

public class PlaybackAuton extends CommandBase {

  private RobotRecorder m_Recorder;
  private DriveTrain m_DriveTrain;

  // values retrieved from robotRecorder
  private double joyX;
  private double joyY;
  

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get values from the recording
    joyX = m_Recorder.getRobotData("joyX");
    joyY = m_Recorder.getRobotData("joyY");
    // place to adjust values
    double forward = -1 * joyX;
    double turn = joyY;	
    // drive train 
    m_DriveTrain.arcadeDrive(forward, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: make this command end if safety values don't line up or something
    return false;
  }
}
