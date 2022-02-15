// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.LatchState;

public class ClimberLatch extends CommandBase {

	Climber climber = null;
	LatchState state = null;

	public ClimberLatch(Climber climber, LatchState state) {
		this.climber = climber;
		this.state = state;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (state == LatchState.OPEN) {
			climber.latchOpen();
		} else {
			climber.latchClose();
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// empty
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		boolean status = false;
		if (state == LatchState.OPEN) {
			if (climber.getLatchState() == LatchState.OPEN) {
				status = true;
			}
		} else {
			if (climber.getLatchState() == LatchState.CLOSE) {
				status = true;
			}
		}
		return status;
	}
}
