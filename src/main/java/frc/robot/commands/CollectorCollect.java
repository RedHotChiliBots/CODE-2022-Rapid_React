// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class CollectorCollect extends CommandBase {
  /** Creates a new CollectorCollect. */

  Collector collector;
  
  public CollectorCollect(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setCollectorVelocity(CollectorConstants.kCollectorCollectRPMs);
    collector.setRunning(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.stopCollect();
    collector.setRunning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !collector.isRunning();
  }
}
