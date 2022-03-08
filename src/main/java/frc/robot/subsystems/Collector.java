// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DIOChannelConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.PneumaticChannelConstants;

public class Collector extends SubsystemBase {

	// ==============================================================
	// Define Motor
	private final CANSparkMax collectorMotor = new CANSparkMax(
			CANidConstants.kCollectorMotor,
			MotorType.kBrushless);

	// ==============================================================
	// Define PID Controller
	private final SparkMaxPIDController collectorPIDController = collectorMotor.getPIDController();

	// ==============================================================
	// Define Encoder
	private final RelativeEncoder collectorEncoder = collectorMotor.getEncoder();

	// ==============================================================
	// Define Solenoid
	private final DoubleSolenoid collectorArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kCollectorExtend,
			PneumaticChannelConstants.kCollectorRetract);

	// ==============================================================
	// Define Digital Inputs
	private final DigitalInput collectorExiting = new DigitalInput(DIOChannelConstants.kCollectorExiting);

	// ==============================================================
	// Define Library
	private final Library lib = new Library();

	// ==============================================================
	// Define Shuffleboard Tab
	private final ShuffleboardTab collectorTab = Shuffleboard.getTab("Collector");
	private final NetworkTableEntry sbCollectorVel = collectorTab.addPersistent("Collector Velocity", 0).getEntry();
	private final NetworkTableEntry sbSetPoint = collectorTab.addPersistent("Collector SetPoint", 0).getEntry();
	private final NetworkTableEntry sbAtTarget = collectorTab.addPersistent("At Target", false).getEntry();
	private final NetworkTableEntry sbCollectorState = collectorTab.addPersistent("Collector State", "").getEntry();
	private final NetworkTableEntry sbExiting = collectorTab.addPersistent("Exiting", false).getEntry();

	// ==============================================================
	// Define Local Variables
	private double collectorSetPoint = 0.0;
	private boolean running = false;

	public enum ArmState {
		NA,
		DEPLOY,
		STOW
	}

	private volatile ArmState armState = ArmState.NA;

	public enum CollectorState {
		NA,
		EMPTY,
		EXITING
	}

	private volatile CollectorState collectorState = CollectorState.NA;

	public Collector() {
		System.out.println("+++++ Collector Constructor starting +++++");

		// ==============================================================
		// Configure Motor
		collectorMotor.restoreFactoryDefaults();
		collectorMotor.clearFaults();
		collectorMotor.setIdleMode(IdleMode.kBrake);

		// ==============================================================
		// Configure PID Controller
		collectorPIDController.setP(CollectorConstants.kP);
		collectorPIDController.setI(CollectorConstants.kI);
		collectorPIDController.setD(CollectorConstants.kD);
		collectorPIDController.setIZone(CollectorConstants.kIz);
		collectorPIDController.setFF(CollectorConstants.kFF);
		collectorPIDController.setOutputRange(CollectorConstants.kMinOutput, CollectorConstants.kMaxOutput);

		// ==============================================================
		// Update Suffleboard Tab with static data
		// SmartDashboard.putNumber("P Gain", CollectorConstants.kP);
		// SmartDashboard.putNumber("I Gain", CollectorConstants.kI);
		// SmartDashboard.putNumber("D Gain", CollectorConstants.kD);
		// SmartDashboard.putNumber("I Zone", CollectorConstants.kIz);
		// SmartDashboard.putNumber("Feed Forward", CollectorConstants.kFF);
		// SmartDashboard.putNumber("Max Output", CollectorConstants.kMaxOutput);
		// SmartDashboard.putNumber("Min Output", CollectorConstants.kMinOutput);
		// SmartDashboard.putNumber("Set Rotations", 0);

		// ==============================================================
		// Initialize devices before starting
		setCollectorVelocity(0.0);
		armStow();

		System.out.println("----- Collector Constructor finished -----");
	}

	@Override
	public void periodic() {
		// // ==============================================================
		// // read PID coefficients from SmartDashboard
		// double p = SmartDashboard.getNumber("P Gain", 0);
		// double i = SmartDashboard.getNumber("I Gain", 0);
		// double d = SmartDashboard.getNumber("D Gain", 0);
		// double iz = SmartDashboard.getNumber("I Zone", 0);
		// double ff = SmartDashboard.getNumber("Feed Forward", 0);
		// double max = SmartDashboard.getNumber("Max Output", 0);
		// double min = SmartDashboard.getNumber("Min Output", 0);

		// // ==============================================================
		// // if PID coefficients on SmartDashboard have changed, write new values to
		// // controller
		// if (p != collectorPIDController.getP()) {
		// collectorPIDController.setP(p);
		// }
		// if (i != collectorPIDController.getI()) {
		// collectorPIDController.setI(i);
		// }
		// if (d != collectorPIDController.getD()) {
		// collectorPIDController.setD(d);
		// }
		// if (iz != collectorPIDController.getIZone()) {
		// collectorPIDController.setIZone(iz);
		// }
		// if (ff != collectorPIDController.getFF()) {
		// collectorPIDController.setFF(ff);
		// }
		// if ((max != collectorPIDController.getOutputMax()) || (min !=
		// collectorPIDController.getOutputMin())) {
		// collectorPIDController.setOutputRange(min, max);
		// }

		collectorSensorState();

		// ==============================================================
		// Update Shuffleboard Tab with dynamic data
		sbCollectorVel.setDouble(getCollectorVelocity());
		sbSetPoint.setDouble(collectorSetPoint);
		sbAtTarget.setBoolean(atTarget());
		sbCollectorState.setString(collectorState.toString());
		sbExiting.setBoolean(isExiting());
	}

	public boolean isExiting() {
		return !collectorExiting.get();
	}

	public void setCollectorState(CollectorState state) {
		collectorState = state;
	}

	public CollectorState getCollectorState() {
		return collectorState;
	}

	public ArmState getArmState() {
		return armState;
	}

	public void collectorSensorState() {
		if (isExiting()) {
			setCollectorState(CollectorState.EXITING);
		} else {
			setCollectorState(CollectorState.EMPTY);
		}
	}

	public double getCollectorVelocity() {
		return collectorEncoder.getVelocity();
	}

	public void setCollectorVelocity(double rpm) {
		this.collectorSetPoint = lib.Clip(-rpm, CollectorConstants.kMaxRPM, CollectorConstants.kMinRPM);
		collectorPIDController.setReference(collectorSetPoint, ControlType.kVelocity);
	}

	public void stopCollector() {
		collectorMotor.set(0);
	}

	public boolean atTarget() {
		return Math.abs(collectorSetPoint - getCollectorVelocity()) <= CollectorConstants.kVelocityTolerance;
	}

	public void setRunning(boolean r) {
		this.running = r;
	}

	public boolean isRunning() {
		return running;
	}

	public void armDeploy() {
		armDeploy(false);
	}

	public void armDeploy(boolean noWait) {
		Thread thread = new Thread("ArmDeploy") {
			@Override
			public void run() {

				collectorArm.set(Value.kForward);

				try {
					TimeUnit.MILLISECONDS.sleep(CollectorConstants.kArmDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("ArmDeploy sleep exception", true);
				}

				armState = ArmState.DEPLOY;
			}
		};
		thread.start();
	}

	public void armStow() {
		armStow(false);
	}

	public void armStow(boolean noWait) {
		Thread thread = new Thread("ArmDeploy") {
			@Override
			public void run() {

				collectorArm.set(Value.kReverse);

				try {
					TimeUnit.MILLISECONDS.sleep(CollectorConstants.kArmDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("ArmStow sleep exception", true);
				}

				armState = ArmState.STOW;
			}
		};
		thread.start();
	}
}
