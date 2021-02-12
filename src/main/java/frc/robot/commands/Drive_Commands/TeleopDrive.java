/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class TeleopDrive extends CommandBase {

  private final DriveTrain m_DriveTrain;
  private final RobotContainer m_RobotContainer;
  private Joystick m_stick;

  /**
   * Creates a new TelepoDrive.
   */
  public TeleopDrive(DriveTrain subsystem, RobotContainer roboContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_RobotContainer = roboContainer;
    m_DriveTrain = subsystem;
    addRequirements(m_DriveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stick = m_RobotContainer.driver;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -1 * m_stick.getY();
    double turn = m_stick.getX();	
    m_DriveTrain.arcadeDrive(forward, turn);
  }

  public double getZStick(){

    return  m_stick.getZ();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
