// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Shooter.GuardState;
import frc.robot.subsystems.Shooter.PlungerState;
import frc.robot.RobotContainer;

public class ShooterShootTimed extends CommandBase {
	private Shooter shooter = null;
	private RobotContainer robotContainer = null;

	public ShooterShootTimed(Shooter shooter, RobotContainer robotContainer) {
		this.shooter = shooter;
		this.robotContainer = robotContainer;
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// empty
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (shooter.atShootTarget()
				// && shooter.getGuardState() == GuardState.CLOSED
				// && shooter.getPlungerState() == PlungerState.READY
				&& shooter.getShooterState() == ShooterState.CONTROLLED) {

			// shooter.plungerPlunge();
		} else {
			robotContainer.doRumble(robotContainer.operator, RumbleType.kRightRumble);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// empty
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
