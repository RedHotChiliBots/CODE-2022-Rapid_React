// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.Constants.ChassisConstants;

public class RamseteCommandOurs extends RamseteCommand {
  /** Creates a new DriveTrajectory. */
  Chassis chassis;

  public RamseteCommandOurs(Chassis chassis, Trajectory trajectory) {
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
        chassis::driveTankVolts,
        chassis);

    this.chassis = chassis;
    addRequirements(chassis);
  }

}
