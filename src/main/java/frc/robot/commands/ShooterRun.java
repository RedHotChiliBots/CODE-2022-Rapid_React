// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Hopper.HopperState;

public class ShooterRun extends CommandBase {
  /** Creates a new ShooterShoot. */

  private final Shooter shooter;
  private final Hopper hopper;
  private final Feeder feeder;
  private Timer timer = new Timer();

  public ShooterRun(Shooter shooter, Hopper hopper, Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.hopper = hopper;
    this.feeder = feeder;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShootVelocity(ShooterConstants.kShooterRPMs);
    shooter.setRunning(true);
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(hopper.getHopperState() == HopperState.EMPTY && feeder.getFeederState() == FeederState.EMPTY) {
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShoot();
    shooter.setRunning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
