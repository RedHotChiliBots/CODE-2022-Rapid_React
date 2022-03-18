// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDrivePosition extends CommandBase {
  /** Creates a new ChassisDriveDistance. */

  private Chassis chassis = null;
  private double setPoint = 0.0;

  public ChassisDrivePosition(Chassis chassis, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.setPoint = setPoint;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		chassis.drivePosition(setPoint);
	}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.atTarget();
  }
}
