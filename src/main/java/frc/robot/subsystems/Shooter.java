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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.Library;
import frc.robot.Constants.CANidConstants;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

	// ==============================================================
	// Define Motors
	private final CANSparkMax shootRMotor = new CANSparkMax(
			CANidConstants.kShooterRMotor,
			MotorType.kBrushless);

	private final CANSparkMax shootLMotor = new CANSparkMax(
			CANidConstants.kShooterLMotor,
			MotorType.kBrushless);

	// ==============================================================
	// Define PID Controller
	private final SparkMaxPIDController shootRPIDController = shootRMotor.getPIDController();
	private final SparkMaxPIDController shootLPIDController = shootLMotor.getPIDController();

	// ==============================================================
	// Define Encoder
	private final RelativeEncoder shootREncoder = shootRMotor.getEncoder();
	private final RelativeEncoder shootLEncoder = shootLMotor.getEncoder();

	// ==============================================================
	// Define Library
	private final Library lib = new Library();

	// ==============================================================
	// Define Suffleboard Tab
	private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
	private final NetworkTableEntry sbLShootVel = shooterTab.addPersistent("ShootVelocityL", 0).getEntry();
	private final NetworkTableEntry sbRShootVel = shooterTab.addPersistent("ShootVelocityR", 0).getEntry();
	private final NetworkTableEntry sbSetPoint = shooterTab.addPersistent("Shoot SetPoint", 0).getEntry();
	private final NetworkTableEntry sbAtTarget = shooterTab.addPersistent("At Target", false).getEntry();
	private final NetworkTableEntry sbShooterState = shooterTab.addPersistent("Shooter State", "").getEntry();
	private final NetworkTableEntry sbEntering = shooterTab.addPersistent("Entering", false).getEntry();
	private final NetworkTableEntry sbExiting = shooterTab.addPersistent("Exiting", false).getEntry();

	private final NetworkTableEntry sbP = shooterTab.addPersistent("P Gain", ShooterConstants.kShootP).getEntry();
	private final NetworkTableEntry sbI = shooterTab.addPersistent("I Gain", ShooterConstants.kShootI).getEntry();
	private final NetworkTableEntry sbD = shooterTab.addPersistent("D Gain", ShooterConstants.kShootD).getEntry();
	private final NetworkTableEntry sbIz = shooterTab.addPersistent("I Zone", ShooterConstants.kShootIz).getEntry();
	private final NetworkTableEntry sbFF = shooterTab.addPersistent("Feed Forward", ShooterConstants.kShootFF).getEntry();
	private final NetworkTableEntry sbMaxOut = shooterTab.addPersistent("Max Output", ShooterConstants.kShootMaxOutput).getEntry();
	private final NetworkTableEntry sbMinOut = shooterTab.addPersistent("Min Output", ShooterConstants.kShootMinOutput).getEntry();
	private final NetworkTableEntry sbRot = shooterTab.addPersistent("Set Rotations", 0).getEntry();

	// ==============================================================
	// Define local variables
	private double shootSetPoint = 0.0;
	private boolean running = false;
	private boolean shootNow = false;

	public Shooter() {
		System.out.println("+++++ Shooter Constructor starting +++++");

		// ==============================================================
		// Configure Motors
		shootRMotor.restoreFactoryDefaults();
		shootRMotor.clearFaults();
		shootRMotor.setInverted(false);
		shootRMotor.setIdleMode(IdleMode.kBrake);

		shootLMotor.restoreFactoryDefaults();
		shootLMotor.clearFaults();
		shootLMotor.setInverted(false);
		shootLMotor.setIdleMode(IdleMode.kBrake);

		// ==============================================================
		// Configure PID Controller
		shootRPIDController.setP(ShooterConstants.kShootP);
		shootRPIDController.setI(ShooterConstants.kShootI);
		shootRPIDController.setD(ShooterConstants.kShootD);
		shootRPIDController.setIZone(ShooterConstants.kShootIz);
		shootRPIDController.setFF(ShooterConstants.kShootFF);
		shootRPIDController.setOutputRange(ShooterConstants.kShootMinOutput, ShooterConstants.kShootMaxOutput);

		shootLPIDController.setP(ShooterConstants.kShootP);
		shootLPIDController.setI(ShooterConstants.kShootI);
		shootLPIDController.setD(ShooterConstants.kShootD);
		shootLPIDController.setIZone(ShooterConstants.kShootIz);
		shootLPIDController.setFF(ShooterConstants.kShootFF);
		shootLPIDController.setOutputRange(ShooterConstants.kShootMinOutput, ShooterConstants.kShootMaxOutput);

		shootLMotor.follow(shootRMotor, true);

		// ==============================================================
		// Display PID coefficients on SmartDashboard
		// SmartDashboard.putNumber("P Gain", ShooterConstants.kShootP);
		// SmartDashboard.putNumber("I Gain", ShooterConstants.kShootI);
		// SmartDashboard.putNumber("D Gain", ShooterConstants.kShootD);
		// SmartDashboard.putNumber("I Zone", ShooterConstants.kShootIz);
		// SmartDashboard.putNumber("Feed Forward", ShooterConstants.kShootFF);
		// SmartDashboard.putNumber("Max Output", ShooterConstants.kShootMaxOutput);
		// SmartDashboard.putNumber("Min Output", ShooterConstants.kShootMinOutput);
		// SmartDashboard.putNumber("Set Rotations", 0);

		// ==============================================================
		// Initialize devices before starting
		setShootVelocity(0.0);

		System.out.println("----- Shooter Constructor finished -----");
	}

	@Override
	public void periodic() {
		// read PID coefficients from SmartDashboard
		double p = sbP.getDouble(0);
		double i = sbI.getDouble(0);
		double d = sbD.getDouble(0);
		double iz = sbIz.getDouble(0);
		double ff = sbFF.getDouble(0);
		double max = sbMaxOut.getDouble(0);
		double min = sbMinOut.getDouble(0);

		// if PID coefficients on SmartDashboard have changed from PID setting, then
		// write new values to PID controller
		if (p != shootRPIDController.getP()) {
		shootRPIDController.setP(p);
		}
		if (i != shootRPIDController.getI()) {
		shootRPIDController.setI(i);
		}
		if (d != shootRPIDController.getD()) {
		shootRPIDController.setD(d);
		}
		if (iz != shootRPIDController.getIZone()) {
		shootRPIDController.setIZone(iz);
		}
		if (ff != shootRPIDController.getFF()) {
		shootRPIDController.setFF(ff);
		}
		if ((max != shootRPIDController.getOutputMax()) || (min !=
		shootRPIDController.getOutputMin())) {
		shootRPIDController.setOutputRange(min, max);
		}

		sbRShootVel.setDouble(getShootRVelocity());
		sbLShootVel.setDouble(getShootLVelocity());
		sbSetPoint.setDouble(shootSetPoint);
		sbAtTarget.setBoolean(atShootTarget());
	}

	public double getShootRVelocity() {
		return shootREncoder.getVelocity();
	}

	public double getShootLVelocity() {
		return shootLEncoder.getVelocity();
	}

	public void setShootVelocity(double rpm) {
		this.shootSetPoint = lib.Clip(-rpm, ShooterConstants.kMaxShootRPM,
				ShooterConstants.kMinShootRPM);
		shootRPIDController.setReference(shootSetPoint, ControlType.kVelocity);
		// shootMotor.setVoltage(0.8);
	}

	public void setShootVoltage() {
		shootRMotor.set(0.20);
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

	public void setShootNow(boolean s) {
		this.shootNow = s;
	}

	public boolean isShootNow() {
		return shootNow;
	}

	public boolean atShootTarget() {
		return Math.abs(shootSetPoint - getShootRVelocity()) <= ShooterConstants.kShootVelocityTolerance;
	}
}
