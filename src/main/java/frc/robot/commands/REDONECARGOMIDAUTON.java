// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class REDONECARGOMIDAUTON extends SequentialCommandGroup {
	/** Creates a new REDONECARGOAUTON. */
	public REDONECARGOMIDAUTON(Chassis chassis, Collector collector,
			Hopper hopper, Shooter shooter) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				// new SHOOT(shooter, hopper),
				new ParallelRaceGroup(
						new DRIVETRAJANDCOLLECT(chassis, RobotContainer.RedRungSideMid, collector, hopper, shooter),
						new WaitCommand(7.0)),
				new ShooterShoot(shooter, hopper),
				new DrivePosition(chassis, 4.0),
				new ShooterStop(shooter));
	}
}
