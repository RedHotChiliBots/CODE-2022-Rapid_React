// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberInit extends SequentialCommandGroup {

	public ClimberInit(Climber climber) {
		climber.setClimberState(ClimberState.NOTINIT);

		addCommands(
				new ParallelCommandGroup(
						new ClimberLeftInit(climber),
						new ClimberRightInit(climber)),
				new ClimberResetEncoder(climber));
	}
}
