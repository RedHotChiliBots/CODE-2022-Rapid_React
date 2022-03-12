// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;

public class FeederSuckIn extends CommandBase {
  /** Creates a new FeederSuckIn. */

	Feeder feeder = null;

  public FeederSuckIn(Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
		this.feeder = feeder;
		addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		feeder.setFeederVelocity(-FeederConstants.kFeederRPMs);
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feeder.getFeederState() == FeederState.CONTROLLED;
  }
}
