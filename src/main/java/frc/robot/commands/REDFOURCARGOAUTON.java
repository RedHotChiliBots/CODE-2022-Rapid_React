// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class REDFOURCARGOAUTON extends SequentialCommandGroup {
	/** Creates a new RedFourCargoAutonTrajPath. */
	public REDFOURCARGOAUTON(Chassis chassis, Collector collector, Hopper hopper, Shooter shooter) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new ShooterRun(shooter, hopper),
				// new FeederShoot(feeder, hopper, shooter),
				new DRIVETRAJANDCOLLECT(chassis, RobotContainer.RedTermSideOneCargo, collector, hopper, shooter),
				new ShooterRun(shooter, hopper),
				// new FeederShoot(feeder, hopper, shooter),
				new DRIVETRAJANDCOLLECT(chassis, RobotContainer.RedTermSideCargoAndTerm, collector, hopper, shooter),
				new ShooterRun(shooter, hopper),
				// new FeederShoot(feeder, hopper, shooter),
				new ShooterStop(shooter));
	}
}
