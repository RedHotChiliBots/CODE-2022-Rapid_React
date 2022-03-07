// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DIOChannelConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

	// ==============================================================
	// Define Motors
	private final CANSparkMax shootMotor = new CANSparkMax(
			CANidConstants.kShooterMotor,
			MotorType.kBrushless);

	// ==============================================================
	// Define PID Controller
	private final SparkMaxPIDController shootPIDController = shootMotor.getPIDController();

	// ==============================================================
	// Define Encoder
	private final RelativeEncoder shootEncoder = shootMotor.getEncoder();

	// ==============================================================
	// Define Library
	private final Library lib = new Library();

	// ==============================================================
	// Define Suffleboard Tab
	private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
	private final NetworkTableEntry sbShootVel = shooterTab.addPersistent("ShootVelocity", 0).getEntry();
	private final NetworkTableEntry sbSetPoint = shooterTab.addPersistent("Shoot SetPoint", 0).getEntry();
	private final NetworkTableEntry sbAtTarget = shooterTab.addPersistent("At Target", false).getEntry();
	private final NetworkTableEntry sbShooterState = shooterTab.addPersistent("Shooter State", "").getEntry();
	private final NetworkTableEntry sbEntering = shooterTab.addPersistent("Entering", false).getEntry();
	private final NetworkTableEntry sbExiting = shooterTab.addPersistent("Exiting", false).getEntry();

	// ==============================================================
	// Define local variables
	private double shootSetPoint = 0.0;
	private boolean running = false;

	public Shooter() {
		System.out.println("+++++ Shooter Constructor starting +++++");

		// ==============================================================
		// Configure Motors
		shootMotor.restoreFactoryDefaults();
		shootMotor.clearFaults();
		shootMotor.setInverted(true);
		shootMotor.setIdleMode(IdleMode.kBrake);

		// ==============================================================
		// Configure PID Controller
		shootPIDController.setP(ShooterConstants.kShootP);
		shootPIDController.setI(ShooterConstants.kShootI);
		shootPIDController.setD(ShooterConstants.kShootD);
		shootPIDController.setIZone(ShooterConstants.kShootIz);
		shootPIDController.setFF(ShooterConstants.kShootFF);
		shootPIDController.setOutputRange(ShooterConstants.kShootMinOutput, ShooterConstants.kShootMaxOutput);

		// ==============================================================
		// Display PID coefficients on SmartDashboard
		// SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
		// SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
		// SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
		// SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
		// SmartDashboard.putNumber("Feed Forward", ShooterConstants.kFF);
		// SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
		// SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
		// SmartDashboard.putNumber("Set Rotations", 0);

		// ==============================================================
		// Initialize devices before starting
		setShootVelocity(0.0);

		System.out.println("----- Shooter Constructor finished -----");
	}

	@Override
	public void periodic() {
		// // read PID coefficients from SmartDashboard
		// double p = SmartDashboard.getNumber("P Gain", 0);
		// double i = SmartDashboard.getNumber("I Gain", 0);
		// double d = SmartDashboard.getNumber("D Gain", 0);
		// double iz = SmartDashboard.getNumber("I Zone", 0);
		// double ff = SmartDashboard.getNumber("Feed Forward", 0);
		// double max = SmartDashboard.getNumber("Max Output", 0);
		// double min = SmartDashboard.getNumber("Min Output", 0);

		// // if PID coefficients on SmartDashboard have changed from PID setting, then
		// // write new values to PID controller
		// if (p != shootPIDController.getP()) {
		// shootPIDController.setP(p);
		// }
		// if (i != shootPIDController.getI()) {
		// shootPIDController.setI(i);
		// }
		// if (d != shootPIDController.getD()) {
		// shootPIDController.setD(d);
		// }
		// if (iz != shootPIDController.getIZone()) {
		// shootPIDController.setIZone(iz);
		// }
		// if (ff != shootPIDController.getFF()) {
		// shootPIDController.setFF(ff);
		// }
		// if ((max != shootPIDController.getOutputMax()) || (min !=
		// shootPIDController.getOutputMin())) {
		// shootPIDController.setOutputRange(min, max);
		// }

		sbShootVel.setDouble(getShootVelocity());
		sbSetPoint.setDouble(shootSetPoint);
		sbAtTarget.setBoolean(atShootTarget());
	}

	public double getShootVelocity() {
		return shootEncoder.getVelocity();
	}

	public void setShootVelocity(double rpm) {
		this.shootSetPoint = lib.Clip(-rpm, ShooterConstants.kMaxShootRPM, ShooterConstants.kMinShootRPM);
		shootPIDController.setReference(shootSetPoint, ControlType.kVelocity);
	}

	public void stopShoot() {
		setShootVelocity(0.0);
	}

	public void setRunning(boolean r) {
		this.running = r;
	}

	public boolean isRunning() {
		return running;
	}

	public boolean atShootTarget() {
		return Math.abs(shootSetPoint - getShootVelocity()) <= ShooterConstants.kShootVelocityTolerance;
	}
}
