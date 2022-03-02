// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.LatchState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberMidRungClimb extends SequentialCommandGroup {

	public ClimberMidRungClimb(Climber climber, Chassis chassis) {

		/**
		 * This command assumes robot has cleared the Low Rung and
		 * is positioned to intercept the Mid Rung driving forward.
		 */

		addCommands(
			// With latch Open and Climber extended, drive in reverse until Chassis Pitch
			new ClimberInit(climber),
			new ClimberLatch(climber, LatchState.OPEN),
			new ClimberGoTo(climber, ClimberConstants.kEngageMidRung),
//			new ChassisDriveIntoRung(chassis),
			// Climb to Rung, Close the Latch, and release the Climber
			new ClimberGoTo(climber, ClimberConstants.kPullUpLatch),
			new ClimberLatch(climber, LatchState.CLOSE),
			new ClimberGoTo(climber, ClimberConstants.kPullUpClear));
	}
}
