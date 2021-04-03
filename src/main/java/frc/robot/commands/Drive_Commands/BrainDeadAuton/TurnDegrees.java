// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands.BrainDeadAuton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnDegrees extends CommandBase {
  private DriveTrain _DriveTrain;
  private double degrees;

  /** Creates a new TurnDegrees. */
  public TurnDegrees(DriveTrain driveTrain, double degrees) {
    this.degrees = degrees;
    _DriveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _DriveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fraction = this.degrees - _DriveTrain.getHeading();
    
    double turn = Math.signum(fraction);

    System.out.println(_DriveTrain.getHeading());
    double throttle = 0.3;

    if(Math.abs(fraction) < 10){
      throttle = 0.20;
    }
    
    _DriveTrain.arcadeDrive(0, turn*throttle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _DriveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(_DriveTrain.getHeading() - this.degrees) < 1) & (Math.abs(_DriveTrain.gyroRate()) < 0.1);
  }
}
