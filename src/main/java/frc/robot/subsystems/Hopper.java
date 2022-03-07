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
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {

  // ==============================================================
  // Define Motor
  private final CANSparkMax hopperMotor = new CANSparkMax(
      CANidConstants.kHopperMotor,
      MotorType.kBrushless);

  // ==============================================================
  // Define PID Controller
  private final SparkMaxPIDController hopperPIDController = hopperMotor.getPIDController();

  // ==============================================================
  // Define Encoder
  private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();

  // ==============================================================
  // Define Digital Inputs
  private final DigitalInput hopperEntering = new DigitalInput(DIOChannelConstants.kHopperEntering);
  private final DigitalInput hopperExiting = new DigitalInput(DIOChannelConstants.kHopperExiting);

  // ==============================================================
  // Define Library
  private final Library lib = new Library();

  // ==============================================================
  // Define Shuffleboard Tab
  private final ShuffleboardTab hopperTab = Shuffleboard.getTab("Hopper");
  private final NetworkTableEntry sbHopperVel = hopperTab.addPersistent("Hopper Velocity", 0).getEntry();
  private final NetworkTableEntry sbSetPoint = hopperTab.addPersistent("Hopper SetPoint", 0).getEntry();
  private final NetworkTableEntry sbAtTarget = hopperTab.addPersistent("At Target", false).getEntry();
  private final NetworkTableEntry sbHopperState = hopperTab.addPersistent("Hopper State", "").getEntry();
  private final NetworkTableEntry sbEntering = hopperTab.addPersistent("Entering", false).getEntry();
  private final NetworkTableEntry sbExiting = hopperTab.addPersistent("Exiting", false).getEntry();

  // ==============================================================
  // Define Local Variables
  private double hopperSetPoint = 0.0;
  private boolean running = false;

  public enum HopperState {
    NA,
    EMPTY,
    ENTERING,
    CONTROLLED,
    EXITING
  }

  private volatile HopperState hopperState = HopperState.NA;

  public Hopper() {
    System.out.println("+++++ Hopper Constructor starting +++++");

    // ==============================================================
    // Configure Motor
    hopperMotor.restoreFactoryDefaults();
    hopperMotor.clearFaults();
    hopperMotor.setIdleMode(IdleMode.kBrake);

    // ==============================================================
    // Configure PID Controller
    hopperPIDController.setP(HopperConstants.kP);
    hopperPIDController.setI(HopperConstants.kI);
    hopperPIDController.setD(HopperConstants.kD);
    hopperPIDController.setIZone(HopperConstants.kIz);
    hopperPIDController.setFF(HopperConstants.kFF);
    hopperPIDController.setOutputRange(HopperConstants.kMinOutput, HopperConstants.kMaxOutput);

    // ==============================================================
    // Update Suffleboard Tab with static data
    // SmartDashboard.putNumber("P Gain", HopperConstants.kP);
    // SmartDashboard.putNumber("I Gain", HopperConstants.kI);
    // SmartDashboard.putNumber("D Gain", HopperConstants.kD);
    // SmartDashboard.putNumber("I Zone", HopperConstants.kIz);
    // SmartDashboard.putNumber("Feed Forward", HopperConstants.kFF);
    // SmartDashboard.putNumber("Max Output", HopperConstants.kMaxOutput);
    // SmartDashboard.putNumber("Min Output", HopperConstants.kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0);

    // ==============================================================
    // Initialize devices before starting
    setHopperVelocity(0.0);

    System.out.println("----- Hopper Constructor finished -----");
  }

  @Override
  public void periodic() {
    // ==============================================================
    // read PID coefficients from SmartDashboard
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
    // if (p != hopperPIDController.getP()) {
    //   hopperPIDController.setP(p);
    // }
    // if (i != hopperPIDController.getI()) {
    //   hopperPIDController.setI(i);
    // }
    // if (d != hopperPIDController.getD()) {
    //   hopperPIDController.setD(d);
    // }
    // if (iz != hopperPIDController.getIZone()) {
    //   hopperPIDController.setIZone(iz);
    // }
    // if (ff != hopperPIDController.getFF()) {
    //   hopperPIDController.setFF(ff);
    // }
    // if ((max != hopperPIDController.getOutputMax()) || (min != hopperPIDController.getOutputMin())) {
    //   hopperPIDController.setOutputRange(min, max);
    // }

    // ==============================================================
    // Update Shuffleboard Tab with dynamic data
    sbHopperVel.setDouble(getHopperVelocity());
    sbSetPoint.setDouble(hopperSetPoint);
    sbAtTarget.setBoolean(atTarget());
    sbHopperState.setString(hopperState.toString());
    sbEntering.setBoolean(isEntering());
    sbExiting.setBoolean(isExiting());
  }

  public boolean isEntering() {
    return hopperEntering.get();
  }

  public boolean isExiting() {
    return hopperExiting.get();
  }

  public void setHopperState(HopperState state) {
    hopperState = state;
  }

	public void hopperSensorState() {
		if(isEntering() == false && isExiting() == false) {
			setHopperState(HopperState.EMPTY);
		} else if (isEntering() == true && isExiting() == false) {
			setHopperState(HopperState.ENTERING);
		} else if (isEntering() == false && isExiting() == true) {
			setHopperState(HopperState.EXITING);
		} else if (isEntering() == true && isExiting() == true) {
			setHopperState(HopperState.CONTROLLED);
		} else {
			setHopperState(HopperState.NA);
		}
	}

  public HopperState getHopperState() {
    return hopperState;
  }

  public double getHopperVelocity() {
    return hopperEncoder.getVelocity();
  }

  public void setHopperVelocity(double rpm) {
    this.hopperSetPoint = lib.Clip(-rpm, HopperConstants.kMaxRPM, HopperConstants.kMinRPM);
    hopperPIDController.setReference(hopperSetPoint, ControlType.kVelocity);
  }

  public void stopHopper() {
    hopperMotor.set(0);
  }

  public boolean atTarget() {
    return Math.abs(hopperSetPoint - getHopperVelocity()) <= HopperConstants.kVelocityTolerance;
  }

  public void setRunning(boolean r) {
    this.running = r;
  }

  public boolean isRunning() {
    return running;
  }
}
