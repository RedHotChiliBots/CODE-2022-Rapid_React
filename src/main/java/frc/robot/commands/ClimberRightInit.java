// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimberRightInit extends CommandBase {

  Climber climber = null;
  DecimalFormat df = new DecimalFormat("#0.0000");
  double time = 0.0;
  Timer initTimer = new Timer();
  CANSparkMax rightMotor = null;
  RelativeEncoder rightEncoder = null;

  public ClimberRightInit(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
//    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		df.setRoundingMode(RoundingMode.CEILING);

    rightMotor = climber.getRightMotor();
    rightEncoder = climber.getRightEncoder();

		time = 0.0;
		initTimer.reset();

		System.out.println("climberInit right start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		rightMotor.set(-ClimberConstants.kInitSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
				rightMotor.set(0.0);
				rightEncoder.setPosition(0.0);

				time = initTimer.get();
				System.out.println(df.format(time) + " climberInit right done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    return climber.getClimberState() == ClimberState.INIT;
    return climber.getRightLimit();
  }
}
