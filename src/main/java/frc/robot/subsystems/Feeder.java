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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DIOChannelConstants;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
  
	private final CANSparkMax feederMotor = new CANSparkMax(
			CANidConstants.kShooterLMotor,
			MotorType.kBrushless);

	private final SparkMaxPIDController feederPIDController = feederMotor.getPIDController();

	private final RelativeEncoder feederEncoder = feederMotor.getEncoder();

	// ==============================================================
	// Define Digital Inputs
	private final DigitalInput feederEntering = new DigitalInput(DIOChannelConstants.kFeederEntering);
	private final DigitalInput feederExiting = new DigitalInput(DIOChannelConstants.kFeederExiting);

	// ==============================================================
	// Define Suffleboard Tab
	private final ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
	private final NetworkTableEntry sbFeederVel = feederTab.addPersistent("FeederVelocity", 0).getEntry();
	private final NetworkTableEntry sbSetPoint = feederTab.addPersistent("Feeder SetPoint", 0).getEntry();
	private final NetworkTableEntry sbAtTarget = feederTab.addPersistent("At Target", false).getEntry();
	private final NetworkTableEntry sbShooterState = feederTab.addPersistent("Feeder State", "").getEntry();
	private final NetworkTableEntry sbEntering = feederTab.addPersistent("Entering", false).getEntry();
	private final NetworkTableEntry sbExiting = feederTab.addPersistent("Exiting", false).getEntry();

	private final Library lib = new Library();
	
	private double feederSetPoint = 0.0;
	private boolean running = false;

	public enum FeederState {
		NA, EMPTY, ENTERING, CONTROLLED, EXITING
	}

	private volatile FeederState feederState = FeederState.NA;

  public Feeder() {
		System.out.println("+++++ Feeder Constructor starting +++++");

		feederMotor.restoreFactoryDefaults();
		feederMotor.clearFaults();
		feederMotor.setIdleMode(IdleMode.kBrake);
		feederMotor.setInverted(true);

		// ==============================================================
		// Configure PID Controller
		feederPIDController.setP(FeederConstants.kFeederP);
		feederPIDController.setI(FeederConstants.kFeederI);
		feederPIDController.setD(FeederConstants.kFeederD);
		feederPIDController.setIZone(FeederConstants.kFeederIz);
		feederPIDController.setFF(FeederConstants.kFeederFF);
		feederPIDController.setOutputRange(FeederConstants.kFeederMinOutput, FeederConstants.kFeederMaxOutput);

		setFeederVelocity(0.0);

		System.out.println("----- Feeder Constructor finished -----");
	}

  @Override
  public void periodic() {
		feederSensorState();
    
		sbSetPoint.setDouble(feederSetPoint);
		sbFeederVel.setDouble(getFeederVelocity());
		sbAtTarget.setBoolean(atFeederTarget());
		sbShooterState.setString(feederState.toString());
		sbEntering.setBoolean(isEntering());
		sbExiting.setBoolean(isExiting());
  }

	public boolean isEntering() {
		return !feederEntering.get();
	}

	public boolean isExiting() {
		return !feederExiting.get();
	}

	public void setFeederState(FeederState state) {
		feederState = state;
	}

	public FeederState getFeederState() {
		return feederState;
	}

	public void feederSensorState() {
		if(isEntering() == false && isExiting() == false) {
			setFeederState(FeederState.EMPTY);
		} else if (isEntering() == true && isExiting() == false) {
			setFeederState(FeederState.ENTERING);
		} else if (isEntering() == false && isExiting() == true) {
			setFeederState(FeederState.EXITING);
		} else if (isEntering() == true && isExiting() == true) {
			setFeederState(FeederState.CONTROLLED);
		} else {
			setFeederState(FeederState.NA);
		}
	}

	public double getFeederVelocity() {
		return feederEncoder.getVelocity();
	}

	public void setFeederVelocity(double rpm) {
		this.feederSetPoint = lib.Clip(-rpm, FeederConstants.kMaxFeederRPM, FeederConstants.kMinFeederRPM);
		feederPIDController.setReference(feederSetPoint, ControlType.kVelocity);
	}

	public void stopFeeder() {
		setFeederVelocity(0.0);
	}

	public void setRunning(boolean r) {
		this.running = r;
	}

	public boolean isRunning() {
		return running;
	}

	public boolean atFeederTarget() {
		return Math.abs(feederSetPoint - getFeederVelocity()) <= FeederConstants.kFeederVelocityTolerance;
	}
}
