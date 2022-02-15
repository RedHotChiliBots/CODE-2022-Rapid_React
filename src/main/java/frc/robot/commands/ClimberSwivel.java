// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.SwivelState;

public class ClimberSwivel extends CommandBase {
	private Climber climber = null;
	private SwivelState state = null;

	public ClimberSwivel(Climber climber, SwivelState state) {
		this.climber = climber;
		this.state = state;

		addRequirements(climber);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (state == SwivelState.SWIVEL) {
			climber.climberSwivel();
		} else {
			climber.climberPerpendicular();
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
		if (state == SwivelState.SWIVEL) {
			if (climber.getSwivelState() == SwivelState.SWIVEL) {
				status = true;
			}
		} else {
			if (climber.getSwivelState() == SwivelState.PERPENDICULAR) {
				status = true;
			}
		}
		return status;
	}
}
