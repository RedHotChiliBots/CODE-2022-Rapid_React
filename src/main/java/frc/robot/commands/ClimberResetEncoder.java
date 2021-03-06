// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;

public class ClimberResetEncoder extends CommandBase {

	Climber climber = null;

	CANSparkMax leftMotor = null;
	RelativeEncoder leftEncoder = null;
	CANSparkMax rightMotor = null;
	RelativeEncoder rightEncoder = null;

	public ClimberResetEncoder(Climber climber) {
		this.climber = climber;
		// Use addRequirements() here to declare subsystem dependencies.
		// addRequirements(climber);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.leftMotor = climber.getLeftMotor();
		this.rightMotor = climber.getRightMotor();

		System.out.println("climberInit configure motors");
		// Group the left and right motors
		rightMotor.follow(leftMotor, true); // invert direction of right motor

		climber.climbPosition(0.0);

		climber.setClimberState(ClimberState.INIT);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return climber.getClimberState() == ClimberState.INIT;
		return true;
	}
}
