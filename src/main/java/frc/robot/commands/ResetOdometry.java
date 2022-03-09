// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ResetOdometry extends CommandBase {
  /** Creates a new ResetOdometry. */

  Chassis chassis = null;
  Trajectory trajectory = null;

  public ResetOdometry(Chassis chassis, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.trajectory = trajectory;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
