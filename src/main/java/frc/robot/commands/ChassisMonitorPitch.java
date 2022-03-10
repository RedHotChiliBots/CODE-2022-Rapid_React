// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisMonitorPitch extends CommandBase {

	Chassis chassis = null;

	private boolean finish = false;

	public ChassisMonitorPitch(Chassis chassis) {
		this.chassis = chassis;
		// Use addRequirements() here to declare subsystem dependencies.
		// addRequirements(chassis);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (chassis.getMaxPitch() - chassis.getMinPitch() < 4.0) {
			finish = true;
		} else if (chassis.getIsPitchIncreasing() & chassis.getPitch() > 0.0) {
			finish = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return climber.getClimberState() == ClimberState.INIT;
		return finish;
	}
}
