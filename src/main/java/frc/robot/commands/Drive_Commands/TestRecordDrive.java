// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotRecorder;
import frc.robot.subsystems.DriveTrain;

public class TestRecordDrive extends CommandBase {

  private RobotRecorder m_Recorder;
  private DriveTrain m_DriveTrain;
  private RobotContainer m_RobotContainer;

  private Joystick m_stick;

  /** Creates a new TestRecordDrive. */
  public TestRecordDrive(RobotRecorder _rRecorder, DriveTrain _dTrain, RobotContainer roboContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Recorder = _rRecorder;
    m_DriveTrain = _dTrain;
    m_RobotContainer = roboContainer;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stick = m_RobotContainer.driver;
    m_Recorder.startRecording();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // same as teleop drive but the values get recorded
    double forward = -1 * m_stick.getY();
    double turn = m_stick.getX();	
    m_DriveTrain.arcadeDrive(forward, turn);

    //record the values

    m_Recorder.setRobotData("joyX",  m_stick.getX());
    m_Recorder.setRobotData("joyY",  m_stick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
