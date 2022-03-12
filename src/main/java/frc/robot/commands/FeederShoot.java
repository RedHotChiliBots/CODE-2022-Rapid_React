// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Hopper.HopperState;

public class FeederShoot extends CommandBase {
  /** Creates a new FeederShoot. */
	private Feeder feeder = null;
  private Hopper hopper = null;
  private Shooter shooter = null;
  private Timer timer = new Timer();

  public FeederShoot(Feeder feeder, Hopper hopper, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
		this.feeder = feeder;
    this.hopper = hopper;
    this.shooter = shooter;
		addRequirements(feeder);
  }

  // Caed when the command is initially schedullled.
  @Override
  public void initialize() {
    shooter.setShootNow(true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		if(shooter.atShootTarget()) {
			feeder.setFeederVelocity(FeederConstants.kFeederRPMs);
		
    	if(!(hopper.getHopperState() == HopperState.EMPTY && feeder.getFeederState() == FeederState.EMPTY)) {
      	timer.reset();
    	}
		}
		
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShootNow(false);
		feeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > ShooterConstants.kTimeShootAfterEmpty;
  }
}
