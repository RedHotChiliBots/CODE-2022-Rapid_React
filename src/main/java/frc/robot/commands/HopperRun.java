// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Collector.ArmState;

public class HopperRun extends CommandBase {
	/** Creates a new HopperShoot. */

	private final Hopper hopper;
	// private final Feeder feeder;
	private final Collector collector;

	public HopperRun(Hopper hopper, /* Feeder feeder, */ Collector collector) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.hopper = hopper;
		// this.feeder = feeder;
		this.collector = collector;
		addRequirements(hopper);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
		// hopper.setRunning(true);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// if(feeder.getFeederState() == FeederState.CONTROLLED) {
		// if(hopper.getHopperState() == HopperState.CONTROLLED) {
		// hopper.stopHopper();
		// } else if(hopper.getHopperState() == HopperState.ENTERING ||
		// hopper.getHopperState() == HopperState.EXITING) {
		// hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
		// } else if(hopper.getHopperState() == HopperState.CONTROLLED) {
		// hopper.stopHopper();
		// }else if(collector.getCollectorState() == CollectorState.EXITING) {
		// hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
		// } else if(hopper.getHopperState() == HopperState.EMPTY ||
		// hopper.getHopperState() == HopperState.NA) {
		// hopper.stopHopper();
		// }
		// } else {
		// if(hopper.getHopperState() == HopperState.CONTROLLED) {
		// hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
		// } else if(hopper.getHopperState() == HopperState.ENTERING ||
		// hopper.getHopperState() == HopperState.EXITING) {
		// hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
		// } else if(collector.getCollectorState() == CollectorState.EXITING) {
		// hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
		// } else if(hopper.getHopperState() == HopperState.EMPTY ||
		// hopper.getHopperState() == HopperState.NA) {
		// hopper.stopHopper();
		// }
		// }

		// if (hopper.isExiting() && !hopper.isEntering()) {
		// 	hopper.stopHopper();
		// } else 
		if (hopper.isEntering() && collector.getArmState() == ArmState.DEPLOY) {
			hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
		} else {
			hopper.stopHopper();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		hopper.stopHopper();
		hopper.setRunning(false);
		// collector.stopCollector();
		// collector.armStow();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return hopper.isExiting() && !hopper.isEntering();
	}
}
