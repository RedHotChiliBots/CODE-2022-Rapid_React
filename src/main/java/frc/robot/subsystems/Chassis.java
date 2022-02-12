// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnalogIOConstants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ChassisConstants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Chassis extends SubsystemBase {

	// ==============================================================
	// Define the left side motors, master and follower
	private final CANSparkMax leftMaster = new CANSparkMax(CANidConstants.kLeftMasterMotor, MotorType.kBrushless);
	private final CANSparkMax leftFollower = new CANSparkMax(CANidConstants.kLeftFollowerMotor, MotorType.kBrushless);

	// Define the right side motors, master and follower
	private final CANSparkMax rightMaster = new CANSparkMax(CANidConstants.kRightMasterMotor, MotorType.kBrushless);
	private final CANSparkMax rightFollower = new CANSparkMax(CANidConstants.kRightFollowerMotor, MotorType.kBrushless);

	private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

	// ==============================================================
	// Identify encoders and PID controllers
	private RelativeEncoder leftEncoder;
	private RelativeEncoder rightEncoder;

	private SparkMaxPIDController leftPIDController;
	private SparkMaxPIDController rightPIDController;

	private double setPoint = 0;

	// ==============================================================
	// Define autonomous support functions
	private DifferentialDriveKinematics kinematics;

	private DifferentialDriveOdometry odometry;

	// Create a voltage constraint to ensure we don't accelerate too fast
	private DifferentialDriveVoltageConstraint autoVoltageConstraint;

	// Create config for trajectory
	private TrajectoryConfig config;
	private TrajectoryConfig configReversed;

	// An example trajectory to follow. All units in meters.
	private Trajectory lineToRendezvousTrajectory;

	// ==============================================================
	// Initialize NavX AHRS board
	// Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	// ==============================================================
	// Identify PDP and PCM
	private final PowerDistribution pdp = new PowerDistribution();
	private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

	// Identify compressor hi and lo sensors
	private final AnalogInput hiPressureSensor = new AnalogInput(AnalogIOConstants.kHiPressureChannel);
	private final AnalogInput loPressureSensor = new AnalogInput(AnalogIOConstants.kLoPressureChannel);

	// ==============================================================
	// Define Shuffleboard data
	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	private NetworkTableEntry sbHeading = chassisTab.addPersistent("Heading", 0).getEntry();
	private NetworkTableEntry sbLeftPos = chassisTab.addPersistent("ML Position", 0).getEntry();
	private NetworkTableEntry sbLeftVel = chassisTab.addPersistent("ML Velocity", 0).getEntry();
	private NetworkTableEntry sbRightPos = chassisTab.addPersistent("MR Position", 0).getEntry();
	private NetworkTableEntry sbRightVel = chassisTab.addPersistent("MR Velocity", 0).getEntry();
	private NetworkTableEntry sbLeftPow = chassisTab.addPersistent("ML Power", 0).getEntry();
	private NetworkTableEntry sbRightPow = chassisTab.addPersistent("MR Power", 0).getEntry();
	private NetworkTableEntry sbPitch = chassisTab.addPersistent("Pitch", 0).getEntry();

	private final ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
	private NetworkTableEntry sbHiPressure = pneumaticsTab.addPersistent("Hi Pressure", 0).getEntry();
	private NetworkTableEntry sbLoPressure = pneumaticsTab.addPersistent("Lo Pressure", 0).getEntry();

	public Chassis() {
		System.out.println("+++++ Chassis Constructor starting ...");

		pdp.clearStickyFaults();

		// ==============================================================
		// Configure the left side motors, master and follower
		leftMaster.restoreFactoryDefaults();
		leftFollower.restoreFactoryDefaults();

		leftMaster.setIdleMode(IdleMode.kBrake);
		leftFollower.setIdleMode(IdleMode.kBrake);

		// Configure the right side motors, master and follower
		rightMaster.restoreFactoryDefaults();
		rightFollower.restoreFactoryDefaults();

		rightMaster.setIdleMode(IdleMode.kBrake);
		rightFollower.setIdleMode(IdleMode.kBrake);

		rightMaster.setInverted(true);
		rightFollower.setInverted(true);

		// Group the left and right motors
		leftFollower.follow(leftMaster);
		rightFollower.follow(rightMaster);

		// ==============================================================
		// Identify PID controller
		leftPIDController = leftMaster.getPIDController();
		rightPIDController = rightMaster.getPIDController();

		leftPIDController.setP(ChassisConstants.kP);
		leftPIDController.setI(ChassisConstants.kI);
		leftPIDController.setD(ChassisConstants.kD);
		leftPIDController.setIZone(ChassisConstants.kIz);
		leftPIDController.setFF(ChassisConstants.kFF);
		leftPIDController.setOutputRange(ChassisConstants.kMinOutput, ChassisConstants.kMaxOutput);

		rightPIDController.setP(ChassisConstants.kP);
		rightPIDController.setI(ChassisConstants.kI);
		rightPIDController.setD(ChassisConstants.kD);
		rightPIDController.setIZone(ChassisConstants.kIz);
		rightPIDController.setFF(ChassisConstants.kFF);
		rightPIDController.setOutputRange(ChassisConstants.kMinOutput, ChassisConstants.kMaxOutput);

		// ==============================================================
		// Identify encoders and PID controllers
		leftEncoder = leftMaster.getEncoder();
		rightEncoder = rightMaster.getEncoder();

		leftEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorIPC);
		rightEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorIPC);

		leftEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);
		rightEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);

		// ==============================================================
		// Define autonomous support functions
		kinematics = new DifferentialDriveKinematics(ChassisConstants.kTrackWidth);

		odometry = new DifferentialDriveOdometry(getAngle());

		// Create a voltage constraint to ensure we don't accelerate too fast
		autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(ChassisConstants.ksVolts, ChassisConstants.kvVoltSecondsPerMeter,
						ChassisConstants.kaVoltSecondsSquaredPerMeter),
				kinematics, 10);

		// Create config for trajectory
		config = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
				ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(kinematics)
						// Apply the voltage constraint
						.addConstraint(autoVoltageConstraint)
						.setReversed(false);

		configReversed = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
				ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(kinematics)
						// Apply the voltage constraint
						.addConstraint(autoVoltageConstraint)
						.setReversed(true);

		lineToRendezvousTrajectory = TrajectoryGenerator.generateTrajectory(
				new Pose2d(0, 1.7, new Rotation2d(0)),
				List.of(new Translation2d(1, 1)),
				new Pose2d(2.9, 3.9, new Rotation2d(0)),
				// Pass config
				config);

		chassisTab.addPersistent("ML Pos Factor", leftEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("MR Pos Factor", rightEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("ML Vel Factor", leftEncoder.getVelocityConversionFactor());
		chassisTab.addPersistent("MR Vel Factor", rightEncoder.getVelocityConversionFactor());

		// Reset the field and encoder positions to zero
		resetFieldPosition(0.0, 0.0);

		System.out.println("----- Chassis Constructor finished ...");
	}

	@Override
	public void periodic() {
		sbHeading.setDouble(getAngle().getDegrees());
		sbLeftPos.setDouble(leftEncoder.getPosition());
		sbLeftVel.setDouble(leftEncoder.getVelocity());
		sbRightPos.setDouble(rightEncoder.getPosition());
		sbRightVel.setDouble(rightEncoder.getVelocity());
		sbLeftPow.setDouble(leftMaster.get());
		sbRightPow.setDouble(rightMaster.get());

		sbPitch.setDouble(getPitch());

		sbHiPressure.setDouble(getHiPressure());
		sbLoPressure.setDouble(getLoPressure());

		// Update field position - for autonomous
		updateOdometry();

		Pose2d pose = odometry.getPoseMeters();
		Translation2d trans = pose.getTranslation();
		Rotation2d rot = pose.getRotation();
	}

	public DifferentialDriveOdometry getOdometry() {
		return odometry;
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	}

	public SparkMaxPIDController getLeftPID() {
		return leftPIDController;
	}

	public SparkMaxPIDController getRightPID() {
		return rightPIDController;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
	}

	public double getDuration(Trajectory t) {
		return t.getTotalTimeSeconds();
	}

	public double getPitch() {
		return ahrs.getPitch();
	}

	public void driveTankVolts(double left, double right) {
		diffDrive.tankDrive(left, right);
	}

	public void driveTank(double left, double right) {
		diffDrive.tankDrive(-left, -right);
	}

	public void driveArcade(double spd, double rot) {
		diffDrive.arcadeDrive(-spd, rot);
	}

	public void resetFieldPosition(double x, double y) {
		ahrs.zeroYaw();
		leftEncoder.setPosition(0.0);
		rightEncoder.setPosition(0.0);
		odometry.resetPosition(new Pose2d(x, y, getAngle()), getAngle());
	}

	/**
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getAngle() {
		// Negating the angle because WPILib gyros are CW positive.
		return Rotation2d.fromDegrees(-ahrs.getYaw());
	}

	/**
	 * Sets the desired wheel speeds.
	 *
	 * @param speeds The desired wheel speeds.
	 */
	// public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
	// double leftFeedforward = 0.0;//
	// m_Feedforward.calculate(speeds.leftMetersPerSecond);
	// double rightFeedforward = 0.0; //
	// m_Feedforward.calculate(speeds.rightMetersPerSecond);

	// double leftOutput =
	// leftPIDController.calculate(leftMaster.getEncoder().getVelocity(),
	// speeds.leftMetersPerSecond);
	// double rightOutput =
	// rightPIDController.calculate(rightMaster.getEncoder().getVelocity(),
	// speeds.rightMetersPerSecond);

	// leftMaster.set(leftOutput + leftFeedforward);
	// rightMaster.set(rightOutput + rightFeedforward);
	// }

	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 *
	 * @param xSpeed Linear velocity in m/s.
	 * @param rot    Angular velocity in rad/s.
	 */
	// @SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double xRot) {
		var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, -xRot));
		leftMaster.set(wheelSpeeds.leftMetersPerSecond);
		rightMaster.set(wheelSpeeds.rightMetersPerSecond);
	}

	/**
	 * Updates the field-relative position.
	 */

	public void updateOdometry() {
		odometry.update(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
	}

	public void driveTrajectory(double left, double right) {
		leftMaster.set(left);
		rightMaster.set(right);
	}

	public void drivePosition(double setPoint) {
		this.setPoint = setPoint;
		SmartDashboard.putNumber("PIDSetpoint", setPoint);
		leftPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
		rightPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
	}

	public boolean atTarget() {
		return Math.abs(setPoint - leftEncoder.getPosition()) <= ChassisConstants.kDistanceTolerance &&
				Math.abs(setPoint - rightEncoder.getPosition()) <= ChassisConstants.kDistanceTolerance;
	}

	public double getLoPressure() {
		return 250.0 * (loPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}

}
