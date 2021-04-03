// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.RollerConstants;

public class Rollers extends SubsystemBase {

  // Roller Motors
  private final TalonSRX inMotor = new TalonSRX(RollerConstants.IN_ROLLER_PORT);
  private final TalonSRX clipMotor1 = new TalonSRX(RollerConstants.CLIP_ROLLER1_PORT);
  private final TalonSRX clipMotor2 = new TalonSRX(RollerConstants.CLIP_ROLLER2_PORT);
  private final TalonSRX outMotor = new TalonSRX(RollerConstants.OUT_ROLLER_PORT);

  /** Creates a new Roller. */
  public Rollers() {}

  // Methods for Roller speeds
  public void setIntakeSpeed(double speed){
    inMotor.set(ControlMode.PercentOutput, speed);
  }
  public void setClipSpeed(double speed){
    clipMotor1.set(ControlMode.PercentOutput, speed);
    clipMotor2.set(ControlMode.PercentOutput, speed);    
  }
  public void setOutputSpeed(double speed){
    outMotor.set(ControlMode.PercentOutput, speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
