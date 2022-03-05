// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.LatchState;
import frc.robot.subsystems.Climber.SwivelState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberHighTravClimb extends SequentialCommandGroup {
	/** Creates a new ClimberHighTravClimb. */
	public ClimberHighTravClimb(Climber climber) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				// Tilt the Climber and extend to Engage High/Traverse Rung
				new ClimberSwivel(climber, SwivelState.SWIVEL),
				new ClimberGoTo(climber, ClimberConstants.kEngageHighTrav),
				// Retract the Climber and Climb until Hooks are engaged
				new ClimberSwivel(climber, SwivelState.PERPENDICULAR),
				new ClimberGoTo(climber, ClimberConstants.kHookHighTrav),
				// Open Latch and Climb to Rung
				new ClimberLatch(climber, LatchState.OPEN),
				new ClimberLeftInit(climber),
				// Close the Latch and clear Climber Hooks
				new ClimberLatch(climber, LatchState.CLOSE),
				new ClimberGoTo(climber, ClimberConstants.kPullUpClear));
	}
}
