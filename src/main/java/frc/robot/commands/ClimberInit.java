// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberInit extends SequentialCommandGroup {
  /** Creates a new CLIMB. */
  public ClimberInit(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
 //     new SequentialCommandGroup(
   addCommands(
        new ParallelCommandGroup(
          new ClimberLeftInit(climber),
          new ClimberRightInit(climber)),
        new ClimberResetEncoder(climber));


        // new ClimberLeftInit(climber),
        // new ClimberMidRungClimb(climber),
        // new ClimberHighTravClimb(climber),
        // new ClimberHighTravClimb(climber));
  }
}
