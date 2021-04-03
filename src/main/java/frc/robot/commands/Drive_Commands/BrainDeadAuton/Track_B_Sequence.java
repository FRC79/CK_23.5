// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands.BrainDeadAuton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Track_B_Sequence extends SequentialCommandGroup {
  private DriveTrain _DriveTrain;
  /** Creates a new Track_B_Sequence. */
  public Track_B_Sequence(DriveTrain driveTrain) {
    _DriveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      /* initial S shape from right to left (first two turns) */
      // get to first turn
      new DriveDistance(_DriveTrain,24),
      new Wait(0.3),
      // turn left 45
      new TurnDegrees(_DriveTrain,-45),
      new Wait(0.3),
      // drive to second turn 
      new DriveDistance(_DriveTrain,84),
      new Wait(0.3),
      // turn right 45
      new TurnDegrees(_DriveTrain,45),
      new Wait(0.3),
      /* drive to end of field */
      //drive to third turn
      new DriveDistance(_DriveTrain,102),
      new Wait(0.3),
      /* Enter the circle at the end of the field */
      // turn right 60
      new TurnDegrees(_DriveTrain,60),
      new Wait(0.3),
      // drive to fourth turn
      new DriveDistance(_DriveTrain,110),
      new Wait(0.3),
      // turn left 90
      new TurnDegrees(_DriveTrain,-90),
      new Wait(0.3),
      // drive past D10
      new DriveDistance(_DriveTrain,60),
      new Wait(0.3),
      // turn left 90
      new TurnDegrees(_DriveTrain,-90),
      new Wait(0.3),
      // drive past D10
      new DriveDistance(_DriveTrain,52),
      new Wait(0.3),
      // turn left 90
      new TurnDegrees(_DriveTrain,-90),
      new Wait(0.3),
      // drive to right side of field
      new DriveDistance(_DriveTrain,94),
      new Wait(0.3),
      // turn right 55
      new TurnDegrees(_DriveTrain,54),
      new Wait(0.3),
      // drive to beginin of field
      new DriveDistance(_DriveTrain,118),
      new Wait(0.3),
      // turn right 45
      new TurnDegrees(_DriveTrain,45),
      new Wait(0.3),
      // drive to front of finish zone zone
      new DriveDistance(_DriveTrain,102),
      new Wait(0.3),
      // turn left 45
      new TurnDegrees(_DriveTrain,-45),
      new Wait(0.3),
      // drive into finish zone
      new DriveDistance(_DriveTrain,36),
      new Wait(0.3)
    );
  }
}
