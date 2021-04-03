// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands.BrainDeadAuton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  private DriveTrain _DriveTrain;
  private double distance;
  /**
   * @param distance the distance in inches the robot shoud travel forwards (can be negitive)
  */
  public DriveDistance(DriveTrain driveTrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    _DriveTrain = driveTrain;
    this.distance = distance - 2 ;
    addRequirements(_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _DriveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftDistance = this.distance - _DriveTrain.leftEncoderDistance();
    double rightDistance = this.distance - _DriveTrain.rightEncoderDistance();

    double leftThrottle = Math.signum(leftDistance);
    double rightThrottle = Math.signum(rightDistance);

/*
    if(leftDistance < 24 && leftDistance > 10 ){
      leftThrottle = ((leftDistance-10)+1)/15;
    }

    if(rightDistance < 24 && rightDistance > 10 ){
      rightThrottle = ((rightDistance-10)+1)/15;
    }*/

    if(Math.abs(leftDistance) < 36){
      leftThrottle *= 0.35;
    }
    if(Math.abs(rightDistance) < 36){
      rightThrottle *= 0.35;
    }

    if(Math.abs(_DriveTrain.leftEncoderDistance()) > Math.abs(this.distance)){
      leftThrottle = 0;
    }
    if(Math.abs(_DriveTrain.rightEncoderDistance()) > Math.abs(this.distance)){
      rightThrottle = 0;
    }
    _DriveTrain.tankDrive(0.5*leftThrottle, 0.5*rightThrottle);

    System.out.println(Math.abs(this.distance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _DriveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(_DriveTrain.leftEncoderDistance()) > Math.abs(this.distance))&&(Math.abs(_DriveTrain.rightEncoderDistance()) > Math.abs(this.distance));
  }
}
