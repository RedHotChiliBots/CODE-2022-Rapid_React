// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper.HopperState;

public class HopperShoot extends CommandBase {
  /** Creates a new FeederShoot. */
  private Hopper hopper = null;
  private Shooter shooter = null;
  private Timer shootTimer = new Timer();
  private Timer clearTimer = new Timer();

  private boolean clearing = false;

  public HopperShoot(Hopper hopper, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.shooter = shooter;
    addRequirements(hopper);
  }

  // Caed when the command is initially schedullled.
  @Override
  public void initialize() {
    // shooter.setShootNow(true);
    clearTimer.start();
    clearTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.atShootTarget()) {
      hopper.setHopperVelocity(HopperConstants.kHopperShootRPMS);
    }
		//  else {
		// 	hopper.setHopperVelocity(0.0);
		// }

    if (shootTimer.hasElapsed(3.0) && hopper.getHopperState() == HopperState.EMPTY && !clearing) {
      clearing = true;
      clearTimer.start();
      clearTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShoot();
    hopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clearTimer.hasElapsed(3.0);
  }
}
