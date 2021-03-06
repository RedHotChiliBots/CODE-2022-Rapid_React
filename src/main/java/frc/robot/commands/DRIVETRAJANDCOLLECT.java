// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DRIVETRAJANDCOLLECT extends ParallelCommandGroup {
  /** Creates a new DRIVETRAJANDCOLLECT. */
  public DRIVETRAJANDCOLLECT(Chassis chassis, Trajectory trajectory, Collector collector,
      Hopper hopper, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveTrajectory(chassis, trajectory),
			new COLLECT(collector, hopper));
        // new SequentialCommandGroup(
        //     new ParallelRaceGroup(new COLLECT(collector, hopper),
        //         new WaitCommand(4.0)),
        //     new COLLECTORSTOWSTOP(collector)));
  }
}
