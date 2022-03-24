// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Collector.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class COLLECTORDEPLOYRUN extends ParallelRaceGroup {
  /** Creates a new COLLECTORDEPLOYRUN. */
  public COLLECTORDEPLOYRUN(Collector collector, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(//new CollectorArm(collector, ArmState.DEPLOY),
    new CollectorRun(collector),
    new HopperRun(hopper, collector));
  }
}
