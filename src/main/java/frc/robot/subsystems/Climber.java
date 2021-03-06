// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DIOChannelConstants;
import frc.robot.Constants.PneumaticChannelConstants;
import frc.robot.Constants.CANidConstants;

public class Climber extends SubsystemBase {

	// ==============================================================
	// Define Solenoids
	private final DoubleSolenoid climbRight = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kClimbRightExtend,
			PneumaticChannelConstants.kClimbRightRetract);
	private final DoubleSolenoid climbLeft = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kClimbLeftExtend,
			PneumaticChannelConstants.kClimbLeftRetract);
	private final DoubleSolenoid climbLatch = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kLatchOpen,
			PneumaticChannelConstants.kLatchClose);

	// ==============================================================
	// Define Digital Inputs
	private final DigitalInput leftLimit = new DigitalInput(DIOChannelConstants.kClimberLeftLimit);
	private final DigitalInput rightLimit = new DigitalInput(DIOChannelConstants.kClimberRightLimit);

	// ==============================================================
	// Define Motors
	private final CANSparkMax climbRightMotor = new CANSparkMax(
			CANidConstants.kClimbLeftMotor,
			MotorType.kBrushless);
	private final CANSparkMax climbLeftMotor = new CANSparkMax(
			CANidConstants.kClimbRightMotor,
			MotorType.kBrushless);

	// ==============================================================
	// Define encoders and PID controller
	private final RelativeEncoder leftEncoder = climbLeftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = climbRightMotor.getEncoder();
	private final SparkMaxPIDController climbPIDController = climbLeftMotor.getPIDController();

	// ==============================================================
	// Define Shuffleboard Tab
	private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
	private final NetworkTableEntry cbLeftPos = climberTab.addPersistent("Left Position", 0.0).getEntry();
	private final NetworkTableEntry cbRightPos = climberTab.addPersistent("Right Position", 0.0).getEntry();
	private final NetworkTableEntry cbSetPoint = climberTab.addPersistent("PID Setpoint", 0.0).getEntry();
	private final NetworkTableEntry cbAtTarget = climberTab.addPersistent("At Target", false).getEntry();
	private final NetworkTableEntry cbClimberState = climberTab.addPersistent("Climber State", "").getEntry();
	private final NetworkTableEntry cbSwivel = climberTab.addPersistent("Swivel State", "").getEntry();
	private final NetworkTableEntry cbLeftPower = climberTab.addPersistent("Left Power", 0.0).getEntry();
	private final NetworkTableEntry cbRightPower = climberTab.addPersistent("Right Power", 0.0).getEntry();
	private final NetworkTableEntry cbLeftAmps = climberTab.addPersistent("Left Amps", 0.0).getEntry();
	private final NetworkTableEntry cbRightAmps = climberTab.addPersistent("Right Amps", 0.0).getEntry();
	private final NetworkTableEntry cbLeftLimit = climberTab.addPersistent("Left Limit", false).getEntry();
	private final NetworkTableEntry cbRightLimit = climberTab.addPersistent("Right Limit", false).getEntry();
	private final NetworkTableEntry cbPitch = climberTab.addPersistent("Pitch", 0.0).getEntry();
	private final NetworkTableEntry cbIsPitchIncreasing = climberTab.addPersistent("Is Pitch Increasing", false).getEntry();

	// ==============================================================
	// Define local variables
	private double setPoint = 0;
	Timer initTimer = new Timer();

	public enum SwivelState {
		NA,
		PERPENDICULAR,
		SWIVEL
	}

	private volatile SwivelState swivelState = SwivelState.NA;

	public enum LatchState {
		NA,
		OPEN,
		CLOSE
	}

	private volatile LatchState latchState = LatchState.NA;

	public enum ClimberState {
		INIT,
		NOTINIT
	}

	private volatile ClimberState climberState = ClimberState.NOTINIT;

	Chassis chassis = null;

	public Climber(Chassis chassis) {

		System.out.println("+++++ Climber Constructor starting +++++");

		this.chassis = chassis;
		// ==============================================================
		// Configure left and right Motors
		climbLeftMotor.restoreFactoryDefaults();
		climbRightMotor.restoreFactoryDefaults();

		climbLeftMotor.clearFaults();
		climbRightMotor.clearFaults();

		climbLeftMotor.setIdleMode(IdleMode.kBrake);
		climbRightMotor.setIdleMode(IdleMode.kBrake);

		climbLeftMotor.setInverted(true);
		climbRightMotor.setInverted(true);

		// ==============================================================
		// Configure left and right Encoders
		leftEncoder.setPositionConversionFactor(ClimberConstants.kPosFactorIPC);
		rightEncoder.setPositionConversionFactor(ClimberConstants.kPosFactorIPC);

		// ==============================================================
		// Configure PID controller
		climbPIDController.setP(ClimberConstants.kP);
		climbPIDController.setI(ClimberConstants.kI);
		climbPIDController.setD(ClimberConstants.kD);
		climbPIDController.setIZone(ClimberConstants.kIz);
		climbPIDController.setFF(ClimberConstants.kFF);
		climbPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

		// ==============================================================
		// Initialize devices before starting
//		climberInit();
		climberPerpendicular();
		latchClose();
		initTimer.start();
		initTimer.reset();

		System.out.println("----- Climber Constructor finished -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		cbLeftPos.setDouble(leftEncoder.getPosition());
		cbRightPos.setDouble(rightEncoder.getPosition());
		cbSetPoint.setDouble(setPoint);
		cbAtTarget.setBoolean(atTarget());
		cbClimberState.setString(getClimberState().toString());
		cbSwivel.setString(swivelState.toString());
		cbLeftPower.setDouble(climbLeftMotor.get());
		cbRightPower.setDouble(climbRightMotor.get());
		cbLeftAmps.setDouble(climbLeftMotor.getOutputCurrent());
		cbRightAmps.setDouble(climbRightMotor.getOutputCurrent());
		cbLeftLimit.setBoolean(getLeftLimit());
		cbRightLimit.setBoolean(getRightLimit());
		cbPitch.setDouble(chassis.getPitch());
		cbIsPitchIncreasing.setBoolean(chassis.getIsPitchIncreasing());
	}

	public SwivelState getSwivelState() {
		return swivelState;
	}

	public LatchState getLatchState() {
		return latchState;
	}

	public ClimberState getClimberState() {
		return climberState;
	}

	public void setClimberState(ClimberState state) {
		climberState = state;
	}

	public boolean atTarget() {
		return Math.abs(setPoint - leftEncoder.getPosition()) <= ClimberConstants.kDistanceTolerance;
	}

	public void climbPosition(double setPoint) {
		this.setPoint = setPoint;
		cbSetPoint.setDouble(setPoint);
		climbPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
	}

	public CANSparkMax getLeftMotor() {
		return climbLeftMotor;
	}

	public CANSparkMax getRightMotor() {
		return climbRightMotor;
	}

	public RelativeEncoder getLeftEncoder() {
		return leftEncoder;
	}

	public RelativeEncoder getRightEncoder() {
		return rightEncoder;
	}

	public boolean getLeftLimit() {
		return !leftLimit.get();
	}

	public boolean getRightLimit() {
		return !rightLimit.get();
	}

	// public void climberInit() {
	// 	// Thread thread = new Thread("ClimberInit") {
	// 	// @Override
	// 	// public void run() {
	// 	boolean timedOut = false;
	// 	boolean leftDone = false;
	// 	boolean rightDone = false;
	// 	double time = 0.0;
	// 	initTimer.reset();
	// 	climberState = ClimberState.NOTINIT;

	// 	DecimalFormat df = new DecimalFormat("#0.0000");
	// 	df.setRoundingMode(RoundingMode.CEILING);

	// 	System.out.println("climberInit pending start");

	// 	climbLeftMotor.set(ClimberConstants.kInitSpeed);
	// 	climbRightMotor.set(-ClimberConstants.kInitSpeed);

	// 	while (!(leftDone && rightDone) && !timedOut) {
	// 		time = initTimer.get();
	// 		System.out.println(df.format(time) +
	// 				" LeftPower: " + climbLeftMotor.get() +
	// 				" RightPower: " + climbRightMotor.get());

	// 		if (!leftDone && getLeftLimit()) {
	// 			climbLeftMotor.set(0.0);
	// 			leftEncoder.setPosition(0.0);
	// 			leftDone = true;

	// 			time = initTimer.get();
	// 			System.out.println(df.format(time) + " climberInit left done");
	// 		}

	// 		if (!rightDone && getRightLimit()) {
	// 			climbRightMotor.set(0.0);
	// 			rightEncoder.setPosition(0.0);
	// 			rightDone = true;

	// 			time = initTimer.get();
	// 			System.out.println(df.format(time) + " climberInit right done");
	// 		}

	// 		if (initTimer.hasElapsed(ClimberConstants.kInitSafety)) {
	// 			timedOut = true;
	// 			if (!leftDone)
	// 				climbLeftMotor.set(0.0);
	// 			if (!rightDone)
	// 				climbRightMotor.set(0.0);

	// 			time = initTimer.get();
	// 			System.out.println(df.format(time) + " climberInit safety abort");
	// 		}

	// 		// yield();
	// 	}

	// 	if (!timedOut) {
	// 		time = initTimer.get();
	// 		System.out.println(df.format(time) + " climberInit configure motors");
	// 		// Group the left and right motors
	// 		climbRightMotor.follow(climbLeftMotor, true); // invert direction of right motor

	// 		setPoint = 0.0;
	// 		climbPosition(setPoint);

	// 		climberState = ClimberState.INIT;
	// 	}
	// 	time = initTimer.get();
	// 	System.out.println(df.format(time) + " climberInit finished");
	// }

	// public void climberInit(boolean noWait) {
	// Thread thread = new Thread("ClimberInit") {
	// @Override
	// public void run() {
	// boolean complete = false;
	// boolean timedOut = false;
	// boolean leftDone = false;
	// boolean rightDone = false;
	// double time = 0.0;
	// initTimer.reset();

	// DecimalFormat df = new DecimalFormat("#0.0000");
	// df.setRoundingMode(RoundingMode.CEILING);

	// System.out.println("climberInit pending start");

	// climbLeftMotor.set(ClimberConstants.kInitSpeed);
	// climbRightMotor.set(ClimberConstants.kInitSpeed);

	// while (!complete) {
	// time = initTimer.get();
	// System.out.println(df.format(time) +
	// " Left: Amps: " + df.format(pdp.getCurrent(PDPChannelConstants.kClimberLeft))
	// + " Power: "
	// + df.format(climbLeftMotor.get()));
	// System.out.println(df.format(time) +
	// " Right: Amps: " +
	// df.format(pdp.getCurrent(PDPChannelConstants.kClimberRight)) + " Power: "
	// + df.format(climbRightMotor.get()));

	// if (initTimer.hasElapsed(ClimberConstants.kInitDelay) || noWait) {

	// time = initTimer.get();
	// System.out.println(df.format(time) + " climberInit starting");

	// time = initTimer.get();
	// System.out.println(df.format(time) +
	// " Left: Amps: " + df.format(pdp.getCurrent(PDPChannelConstants.kClimberLeft))
	// + " Power: " + df.format(climbLeftMotor.get()));
	// System.out.println(df.format(time) +
	// " Right: Amps: " +
	// df.format(pdp.getCurrent(PDPChannelConstants.kClimberRight))
	// + " Power: " + df.format(climbRightMotor.get()));

	// while (!(leftDone && rightDone) && !timedOut) {

	// if (climbLeftMotor.getOutputCurrent() > ClimberConstants.kMaxAmps) {
	// climbLeftMotor.set(0.0);
	// rightEncoder.setPosition(0.0);
	// leftDone = true;

	// time = initTimer.get();
	// System.out.println(df.format(time) + " climberInit left done");
	// }
	// if (climbRightMotor.getOutputCurrent() > ClimberConstants.kMaxAmps) {
	// climbRightMotor.set(0.0);
	// leftEncoder.setPosition(0.0);
	// rightDone = true;

	// time = initTimer.get();
	// System.out.println(df.format(time) + " climberInit right done");
	// }

	// if (initTimer.hasElapsed(ClimberConstants.kInitSafety)) {
	// timedOut = true;
	// climbLeftMotor.set(0.0);
	// climbRightMotor.set(0.0);

	// time = initTimer.get();
	// System.out.println(df.format(time) + " climberInit safety abort");
	// }
	// yield();
	// }
	// complete = true;
	// } else {
	// yield();
	// }
	// }

	// if (!timedOut) {
	// time = initTimer.get();
	// System.out.println(df.format(time) + " climberInit configure motors");
	// // Group the left and right motors
	// climbRightMotor.follow(climbLeftMotor, true); // invert direction of right
	// motor

	// setPoint = 0.0;
	// climbPosition(setPoint);
	// }
	// time = initTimer.get();
	// System.out.println(df.format(time) + " climberInit finished");
	// }
	// };
	// thread.start();
	// }

	public void latchOpen() {
		latchOpen(false);
	}

	public void latchOpen(boolean noWait) {
		Thread thread = new Thread("LatchOpen") {
			@Override
			public void run() {

				climbLatch.set(Value.kForward);

				try {
					TimeUnit.MILLISECONDS.sleep(ClimberConstants.kLatchDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("LatchOpen sleep exception", true);
				}

				latchState = LatchState.OPEN;
			}
		};
		thread.start();
	}

	public void latchClose() {
		latchClose(false);
	}

	public void latchClose(boolean noWait) {
		Thread thread = new Thread("LatchClose") {
			@Override
			public void run() {

				climbLatch.set(Value.kReverse);

				try {
					TimeUnit.MILLISECONDS.sleep(ClimberConstants.kLatchDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("LatchClose sleep exception", true);
				}

				latchState = LatchState.CLOSE;
			}
		};
		thread.start();
	}

	public void climberSwivel() {
		climberSwivel(false);
	}

	public void climberSwivel(boolean noWait) {
		Thread thread = new Thread("ClimbExtend") {
			@Override
			public void run() {

				climbLeft.set(Value.kForward);
				climbRight.set(Value.kForward);

				try {
					TimeUnit.MILLISECONDS.sleep(ClimberConstants.kSwivelDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("ClimbExtend sleep exception", true);
				}

				swivelState = SwivelState.SWIVEL;
			}
		};
		thread.start();
	}

	public void climberPerpendicular() {
		climberPerpendicular(false);
	}

	public void climberPerpendicular(boolean noWait) {
		Thread thread = new Thread("ClimberRetract") {
			@Override
			public void run() {

				climbLeft.set(Value.kReverse);
				climbRight.set(Value.kReverse);

				try {
					TimeUnit.MILLISECONDS.sleep(ClimberConstants.kSwivelDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("ClimbRetract sleep exception", true);
				}

				swivelState = SwivelState.PERPENDICULAR;
			}
		};
		thread.start();
	}
}
