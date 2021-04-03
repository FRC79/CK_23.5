// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands.BrainDeadAuton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Track_A_Sequence extends SequentialCommandGroup {
  private DriveTrain _DriveTrain;
  /** Creates a new Track_A_Sequence. */
  public Track_A_Sequence(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    _DriveTrain = driveTrain;
    // addCommands(new FooCommand(), new BarCommand())
    addCommands(
      /* looping the first (D5) point */
      // going to and past point D5
      new DriveDistance(_DriveTrain,148),
      new Wait(0),
      // turn right 90
      new TurnDegrees(_DriveTrain, 90),
      new Wait(0),
      // get past D5
      new DriveDistance(_DriveTrain,60),
      new Wait(0),
      // turn right 90
      new TurnDegrees(_DriveTrain, 90),
      new Wait(0),
      // get past D5
      new DriveDistance(_DriveTrain,70),
      new Wait(0),
      // turn right 90
      new TurnDegrees(_DriveTrain, 90),
      new Wait(0),
      // get past D5
      new DriveDistance(_DriveTrain,62),
      new Wait(0),
      // turn Right 90 
      new TurnDegrees(_DriveTrain, 90),
      new Wait(0),
      /* looping the second (B8) point */
      // go to and past point B8
      new DriveDistance(_DriveTrain,144),
      new Wait(0),
      // turn left 90
      new TurnDegrees(_DriveTrain, -90),
      new Wait(0),
      // go past B8
      new DriveDistance(_DriveTrain,52),
      new Wait(0),
      // turn left 90
      new TurnDegrees(_DriveTrain, -90),
      new Wait(0),
      // go past B8
      new DriveDistance(_DriveTrain,72),
      new Wait(0),
      // turn left 90
      new TurnDegrees(_DriveTrain, -90),
      new Wait(0),
      // go past B8
      new DriveDistance(_DriveTrain,60),
      new Wait(0),
      // turn left 45 degrees
      new TurnDegrees(_DriveTrain, -45),
      new Wait(0),
      /* looping third (D10) point */
      // go to but not past D10
      new DriveDistance(_DriveTrain,92),
      new Wait(0),
      // turn left 45 degrees
      new TurnDegrees(_DriveTrain, -25),
      new Wait(0),
      // go past D10
      new DriveDistance(_DriveTrain,48),
      new Wait(0),
      // turn left 90 
      new TurnDegrees(_DriveTrain, -90),
      new Wait(0),
      // go past D10
      new DriveDistance(_DriveTrain,58),
      new Wait(0),
      //turn left 90
      new TurnDegrees(_DriveTrain, -82),
      new Wait(0),
      /* go all the way to the finish zone */
      new DriveDistance(_DriveTrain,300)
    );
  }
}
