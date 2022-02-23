// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveForward extends CommandBase {
  /** Creates a new DriveForward. */

  Chassis chassis;
  double left;
  double right;

  public DriveForward(Chassis chassis, double left, double right) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.left = left;
    this.right = right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // empty
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.drive(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
