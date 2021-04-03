// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands.BrainDeadAuton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Track_C_Sequence extends SequentialCommandGroup {
  private DriveTrain _DriveTrain;
  /** Creates a new Track_C_Sequence. */
  public Track_C_Sequence(DriveTrain driveTrain) {
    _DriveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // exit start zone
      new DriveDistance(_DriveTrain,48),
      new Wait(0.3),
      // turn left 90
      new TurnDegrees(_DriveTrain,-90),
      new Wait(0.3),
      // go to A3
      new DriveDistance(_DriveTrain,60),
      new Wait(0.3),
      // turn left ~15 degrees in preperation of going backwards
      new TurnDegrees(_DriveTrain,-15),
      new Wait(0.3),
      // back up to right side of the field
      new DriveDistance(_DriveTrain,132),
      new Wait(0.3),
      // turn left ~75 degrees to back up again
      new TurnDegrees(_DriveTrain,-75),
      new Wait(0.3),
      // back up to align with A6
      new DriveDistance(_DriveTrain,60),
      new Wait(0.3),
      // turn right 90 to face A6
      new TurnDegrees(_DriveTrain,90),
      new Wait(0.3),
      // drive to A6
      new DriveDistance(_DriveTrain,120),
      new Wait(0.3),
      // back up to the right side of the field
      new DriveDistance(_DriveTrain,-120+2),
      new Wait(0.3),
      // turn right 90
      new TurnDegrees(_DriveTrain,90),
      new Wait(0.3),
      // drive to align with A9
      new DriveDistance(_DriveTrain,90),
      new Wait(0.3),
      // turn left 90 to face A9
      new TurnDegrees(_DriveTrain,-90),
      new Wait(0.3),
      // go to A3
      new DriveDistance(_DriveTrain,120),
      new Wait(0.3),
      // back up to align with finish zone
      new DriveDistance(_DriveTrain,-60+2),
      new Wait(0.3),
      // turn right 90 to face finish zone
      new TurnDegrees(_DriveTrain,90),
      new Wait(0.3),
      // enter finish zone
      new DriveDistance(_DriveTrain,60)
    );
  }
}
