// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Hopper.HopperState;

public class HopperSuckIn extends CommandBase {
  /** Creates a new HopperSuckIn. */

	private Hopper hopper = null;
	private Feeder feeder = null;
	private Collector collector = null;

  public HopperSuckIn(Hopper hopper, Feeder feeder, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
		this.hopper = hopper;
		this.feeder = feeder;
		this.collector = collector;
		addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		if(feeder.getFeederState() == FeederState.CONTROLLED) {
      if(hopper.getHopperState() == HopperState.CONTROLLED) {
        hopper.stopHopper();
      } else if(hopper.getHopperState() == HopperState.ENTERING || hopper.getHopperState() == HopperState.EXITING) {
        hopper.setHopperVelocity(-HopperConstants.kHopperRPMs);
      } else if(hopper.getHopperState() == HopperState.CONTROLLED) {
        hopper.stopHopper();
      }else if(collector.getCollectorState() == CollectorState.EXITING) {
        hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
      } else if(hopper.getHopperState() == HopperState.EMPTY || hopper.getHopperState() == HopperState.NA) {
        hopper.stopHopper();
      }
    } else {
      if(hopper.getHopperState() == HopperState.CONTROLLED) {
        hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
      } else if(hopper.getHopperState() == HopperState.ENTERING || hopper.getHopperState() == HopperState.EXITING) {
        hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
      } else if(collector.getCollectorState() == CollectorState.EXITING) {
        hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
      } else if(hopper.getHopperState() == HopperState.EMPTY || hopper.getHopperState() == HopperState.NA) {
        hopper.stopHopper();
      }
    }
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
