// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;

public class HopperRun extends CommandBase {
  /** Creates a new HopperShoot. */

  private final Hopper hopper;

  public HopperRun(Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setHopperVelocity(HopperConstants.kHopperRPMs);
    hopper.setRunning(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    hopper.setRunning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !hopper.isRunning();
  }
}
