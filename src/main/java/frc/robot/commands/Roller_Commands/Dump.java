// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Roller_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Rollers;

public class Dump extends CommandBase {
  private final Rollers m_Rollers;
  /** Creates a new Dump. */
  public Dump(Rollers Roller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Rollers = Roller;
    addRequirements(m_Rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Rollers.setIntakeMotors(1.0); // change these to suit ball jamming
    m_Rollers.setClipLowMotor(1);
    m_Rollers.setClipHighMotor(1);
    m_Rollers.setDumpMotors(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Rollers.setIntakeMotors(0); // change these to suit ball jamming
    m_Rollers.setClipLowMotor(0);
    m_Rollers.setClipHighMotor(0);
    m_Rollers.setDumpMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
