// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;

public class DriveTrajectory extends RamseteCommand {
  /** Creates a new DriveTrajectory. */
  Chassis chassis;
  Trajectory trajectory;

  public DriveTrajectory(Chassis chassis, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(
        trajectory,
        chassis::getPose,
        new RamseteController(ChassisConstants.kRamseteB, ChassisConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            ChassisConstants.ksVolts,
            ChassisConstants.kvVoltSecondsPerMeter,
            ChassisConstants.kaVoltSecondsSquaredPerMeter),
        chassis.kinematics,
        chassis::getWheelSpeeds,
        new PIDController(ChassisConstants.kP, ChassisConstants.kI, ChassisConstants.kD),
        new PIDController(ChassisConstants.kP, ChassisConstants.kI, ChassisConstants.kD),
        // RamseteCommand passes volts to the callback
        chassis::tankDriveVolts,
        chassis);

	this.chassis = chassis;
	this.trajectory = trajectory;
	addRequirements(chassis);
}


  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {

  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  // return false;
  // }
}
