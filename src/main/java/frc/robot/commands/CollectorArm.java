// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.ArmState;

public class CollectorArm extends CommandBase {
	/** Creates a new CollectorArmExtend. */

	Collector collector;
	ArmState armState;
	boolean wait = true;

	public CollectorArm(Collector collector, ArmState armState) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.collector = collector;
		this.armState = armState;
		this.wait = true;
		// addRequirements(collector);
	}

	public CollectorArm(Collector collector, ArmState armState, boolean wait) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.collector = collector;
		this.armState = armState;
		this.wait = wait;
		addRequirements(collector);
	}


	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (armState == ArmState.DEPLOY) {
			collector.armDeploy(wait);
		} else {
			collector.armStow(wait);
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
		// empty
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
