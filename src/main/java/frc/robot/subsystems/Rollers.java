/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RollerConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Rollers extends SubsystemBase {
  // intake victors
  private VictorSPX intakeMotor = new VictorSPX(RollerConstants.INTAKE);
  // ball clip victors
  private final VictorSPX clipLowMotor = new VictorSPX(RollerConstants.CLIP_LOW);
  private final VictorSPX clipHighMotor = new VictorSPX(RollerConstants.CLIP_HIGH);
  // ball dump victors
  private final VictorSPX dumpMotor = new VictorSPX(RollerConstants.DUMPER);

  /**
   * Creates a new BallConveyer.
   */
  public Rollers() {
    
  }

  public void setIntakeMotors(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setClipLowMotor(double speed){
    clipLowMotor.set(ControlMode.PercentOutput, speed);
  }
  public void setClipHighMotor(double speed){
    clipHighMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setDumpMotors(double speed){
    dumpMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
