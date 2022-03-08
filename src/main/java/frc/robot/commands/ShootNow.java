// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Hopper.HopperState;

public class ShootNow extends CommandBase {
  /** Creates a new ShootNow. */
	private final Shooter shooter;
  private final Hopper hopper;
  private final Feeder feeder;

  public ShootNow(Shooter shooter, Hopper hopper, Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
		this.shooter = shooter; 
    this.hopper = hopper;
    this.feeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		shooter.setShootNow(true);
	}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
		shooter.setShootNow(false);
	}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hopper.getHopperState() == HopperState.EMPTY && feeder.getFeederState() == FeederState.EMPTY;
  }
}
