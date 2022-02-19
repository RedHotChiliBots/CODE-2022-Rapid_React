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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private final CANSparkMax injectorMotor = new CANSparkMax(
			CANidConstants.kInjectorMotor,
			MotorType.kBrushless);

	// ==============================================================
	// Define PID Controller
	private final SparkMaxPIDController shootPIDController = shootMotor.getPIDController();
	private final SparkMaxPIDController injectPIDController = injectorMotor.getPIDController();

	// ==============================================================
	// Define Encoder
	private final RelativeEncoder shootEncoder = shootMotor.getEncoder();
	private final RelativeEncoder injectEncoder = injectorMotor.getEncoder();

	// ==============================================================
	// Define Digital Inputs
	private final DigitalInput injectorEntering = new DigitalInput(DIOChannelConstants.kInjectorEntering);
	private final DigitalInput injectorExiting = new DigitalInput(DIOChannelConstants.kInjectorExiting);

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
	private double injectSetPoint = 0.0;
	private boolean running = false;

	Timer plungerTimer = new Timer();
	Timer guardTimer = new Timer();

	public enum InjectorState {
		NA, EMPTY, ENTERING, CONTROLLED
	}

	private volatile InjectorState shooterState = InjectorState.NA;

	public enum GuardState {
		NA, OPEN, CLOSED
	}

	private volatile GuardState guardState = GuardState.NA;

	public enum PlungerState {
		NA, READY, ACTIVE
	}

	private volatile PlungerState plungerState = PlungerState.NA;

	public Shooter() {
		System.out.println("+++++ Shooter Constructor starting +++++");

		// ==============================================================
		// Configure Motors
		shootMotor.restoreFactoryDefaults();
		shootMotor.clearFaults();
		shootMotor.setIdleMode(IdleMode.kBrake);

		injectorMotor.restoreFactoryDefaults();
		injectorMotor.clearFaults();
		injectorMotor.setIdleMode(IdleMode.kBrake);

		// ==============================================================
		// Configure PID Controller
		injectPIDController.setP(ShooterConstants.kShootP);
		injectPIDController.setI(ShooterConstants.kShootI);
		injectPIDController.setD(ShooterConstants.kShootD);
		injectPIDController.setIZone(ShooterConstants.kShootIz);
		injectPIDController.setFF(ShooterConstants.kShootFF);
		injectPIDController.setOutputRange(ShooterConstants.kShootMinOutput, ShooterConstants.kShootMaxOutput);

		shootPIDController.setP(ShooterConstants.kInjectP);
		shootPIDController.setI(ShooterConstants.kInjectI);
		shootPIDController.setD(ShooterConstants.kInjectD);
		shootPIDController.setIZone(ShooterConstants.kInjectIz);
		shootPIDController.setFF(ShooterConstants.kInjectFF);
		shootPIDController.setOutputRange(ShooterConstants.kInjectMinOutput, ShooterConstants.kInjectMaxOutput);

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
		setInjectVelocity(0.0);
		guardTimer.reset();
		guardTimer.start();

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
		sbShooterState.setString(shooterState.toString());
		sbEntering.setBoolean(isEntering());
		sbExiting.setBoolean(isExiting());
	}

	public boolean isEntering() {
		return injectorEntering.get();
	}

	public boolean isExiting() {
		return injectorExiting.get();
	}

	public void setInjectorState(InjectorState state) {
		shooterState = state;
	}

	public InjectorState getInjectorState() {
		return shooterState;
	}

	public double getShootVelocity() {
		return shootEncoder.getVelocity();
	}

	public double getInjectVelocity() {
		return injectEncoder.getVelocity();
	}

	public void setShootVelocity(double rpm) {
		this.shootSetPoint = lib.Clip(-rpm, ShooterConstants.kMaxShootRPM, ShooterConstants.kMinShootRPM);
		shootPIDController.setReference(shootSetPoint, ControlType.kVelocity);
	}

	public void setInjectVelocity(double rpm) {
		this.injectSetPoint = lib.Clip(-rpm, ShooterConstants.kMaxInjectRPM, ShooterConstants.kMinInjectRPM);
		injectPIDController.setReference(injectSetPoint, ControlType.kVelocity);
	}

	public void stopShoot() {
		setShootVelocity(0.0);
	}

	public void stopInject() {
		setInjectVelocity(0.0);
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

	public boolean atInjectTarget() {
		return Math.abs(shootSetPoint - getInjectVelocity()) <= ShooterConstants.kInjectVelocityTolerance;
	}
}
