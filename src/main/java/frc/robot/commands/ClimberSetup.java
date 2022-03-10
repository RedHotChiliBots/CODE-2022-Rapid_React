// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Climber.LatchState;
import frc.robot.subsystems.Collector.ArmState;
import frc.robot.subsystems.Collector.CollectorState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberSetup extends SequentialCommandGroup {

	public ClimberSetup(Climber climber, Collector collector) {

		/**
		 * This command assumes robot has cleared the Low Rung and
		 * is positioned to intercept the Mid Rung driving forward.
		 */

		addCommands(
				// With latch Open and Climber extended, drive in reverse until Chassis Pitch
				new CollectorArm(collector, ArmState.DEPLOY),
				new ClimberInit(climber),
				new ClimberLatch(climber, LatchState.OPEN),
				new ClimberGoTo(climber, ClimberConstants.kEngageMidRung));
	}
}
