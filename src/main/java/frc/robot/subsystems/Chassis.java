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

import com.kauailabs.navx.frc.AHRS;

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

import edu.wpi.first.wpilibj.SPI;
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

public class Chassis extends SubsystemBase {

	// ==============================================================
	// Define the left side motors, master and follower
	private final CANSparkMax leftMaster = new CANSparkMax(
			CANidConstants.kLeftMasterMotor,
			MotorType.kBrushless);
	private final CANSparkMax leftFollower = new CANSparkMax(
			CANidConstants.kLeftFollowerMotor,
			MotorType.kBrushless);

	// Define the right side motors, master and follower
	private final CANSparkMax rightMaster = new CANSparkMax(
			CANidConstants.kRightMasterMotor,
			MotorType.kBrushless);
	private final CANSparkMax rightFollower = new CANSparkMax(
			CANidConstants.kRightFollowerMotor,
			MotorType.kBrushless);

	private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

	// ==============================================================
	// Define encoders and PID controllers
	private final RelativeEncoder leftEncoder = leftMaster.getEncoder();
	private final RelativeEncoder rightEncoder = rightMaster.getEncoder();

	private final SparkMaxPIDController leftPIDController = leftMaster.getPIDController();
	private final SparkMaxPIDController rightPIDController = rightMaster.getPIDController();

	private double setPoint = 0;

	// ==============================================================
	// Define autonomous support functions
	public DifferentialDriveKinematics kinematics;

	private DifferentialDriveOdometry odometry;

	// Create a voltage constraint to ensure we don't accelerate too fast
	private DifferentialDriveVoltageConstraint autoVoltageConstraint;

	// Create config for trajectory
	private TrajectoryConfig config;
	private TrajectoryConfig configReversed;

	// An example trajectory to follow. All units in meters.
	private Trajectory lineToRendezvousTrajectory;
	private Trajectory BlueSideRung;

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
	private final NetworkTableEntry sbLeftPos = chassisTab.addPersistent("ML Position", 0).getEntry();
	private final NetworkTableEntry sbLeftVel = chassisTab.addPersistent("ML Velocity", 0).getEntry();
	private final NetworkTableEntry sbRightPos = chassisTab.addPersistent("MR Position", 0).getEntry();
	private final NetworkTableEntry sbRightVel = chassisTab.addPersistent("MR Velocity", 0).getEntry();
	private final NetworkTableEntry sbLeftPow = chassisTab.addPersistent("ML Power", 0).getEntry();
	private final NetworkTableEntry sbRightPow = chassisTab.addPersistent("MR Power", 0).getEntry();
	private final NetworkTableEntry sbPitch = chassisTab.addPersistent("Pitch", 0).getEntry();
	private final NetworkTableEntry sbAngle = chassisTab.addPersistent("Angle", 0).getEntry();
	private final NetworkTableEntry sbHeading = chassisTab.addPersistent("Heading", 0).getEntry();

	private final ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
	private final NetworkTableEntry sbHiPressure = pneumaticsTab.addPersistent("Hi Pressure", 0).getEntry();
	private final NetworkTableEntry sbLoPressure = pneumaticsTab.addPersistent("Lo Pressure", 0).getEntry();

	public Chassis() {
		System.out.println("+++++ Chassis Constructor starting +++++");

		// ==============================================================
		// Configure PDP
		pdp.clearStickyFaults();

		// ==============================================================
		// Configure the left side motors, master and follower
		leftMaster.restoreFactoryDefaults();
		leftFollower.restoreFactoryDefaults();

		leftMaster.clearFaults();
		leftFollower.clearFaults();

		leftMaster.setIdleMode(IdleMode.kBrake);
		leftFollower.setIdleMode(IdleMode.kBrake);

		// Configure the right side motors, master and follower
		rightMaster.restoreFactoryDefaults();
		rightFollower.restoreFactoryDefaults();

		rightMaster.clearFaults();
		rightFollower.clearFaults();

		rightMaster.setIdleMode(IdleMode.kBrake);
		rightFollower.setIdleMode(IdleMode.kBrake);

		rightMaster.setInverted(true);
		rightFollower.setInverted(true);

		// Group the left and right motors
		leftFollower.follow(leftMaster);
		rightFollower.follow(rightMaster);

		// ==============================================================
		// Configure PID controllers
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
		// Configure encoders
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

		// ==============================================================
		// Add static variables to Shuffleboard
		chassisTab.addPersistent("ML Pos Factor", leftEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("MR Pos Factor", rightEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("ML Vel Factor", leftEncoder.getVelocityConversionFactor());
		chassisTab.addPersistent("MR Vel Factor", rightEncoder.getVelocityConversionFactor());

		// ==============================================================
		// Initialize devices before starting
		resetFieldPosition(0.0, 0.0); // Reset the field and encoder positions to zero

		System.out.println("----- Chassis Constructor finished -----");
	}

	@Override
	public void periodic() {
		sbLeftPos.setDouble(leftEncoder.getPosition());
		sbLeftVel.setDouble(leftEncoder.getVelocity());
		sbRightPos.setDouble(rightEncoder.getPosition());
		sbRightVel.setDouble(rightEncoder.getVelocity());
		sbLeftPow.setDouble(leftMaster.get());
		sbRightPow.setDouble(rightMaster.get());

		sbPitch.setDouble(getPitch());
		sbAngle.setDouble(getAngle().getDegrees());
		sbHeading.setDouble(getHeading());

		sbHiPressure.setDouble(getHiPressure());
		sbLoPressure.setDouble(getLoPressure());

		// Update field position - for autonomous
		resetOdometry(lineToRendezvousTrajectory.getInitialPose());

		Pose2d pose = odometry.getPoseMeters();
		Translation2d trans = pose.getTranslation();
		Rotation2d rot = pose.getRotation();
	}

	public PowerDistribution getPDP() {
		return pdp;
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

	/**
	 * Returns the current robot pitch reported by navX sensor.
	 * 
	 * @see com.kauailabs.navx.frc.AHRS.getPitch()
	 * @return The current pitch value in degrees (-180 to 180).
	 */
	public double getPitch() {
		return ahrs.getPitch();
	}

	// public void driveTankVolts(double left, double right) {
	// diffDrive.tankDrive(left, right);
	// }

	public void driveTankVolts(double leftVolts, double rightVolts) {
		leftMaster.set(leftVolts);
		rightMaster.setVoltage(rightVolts);
		diffDrive.feed();
	}

	public void driveTank(double left, double right) {
		diffDrive.tankDrive(-left, -right);
	}

	public void driveArcade(double spd, double rot) {
		diffDrive.arcadeDrive(-spd, rot);
	}

	public void resetFieldPosition(double x, double y) {
		ahrs.zeroYaw();
		resetEncoders();
		odometry.resetPosition(new Pose2d(x, y, getAngle()), getAngle());
	}

	/**
	 * Returns the "fused" (9-axis) heading.
	 * 
	 * @see com.kauailabs.navx.frc.AHRS.getFusedHeading()
	 * @return Fused Heading in Degrees (range 0-360)
	 * 
	 */
	public double getHeading() {
		return ahrs.getFusedHeading();
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

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(pose, ahrs.getRotation2d());
	}

	public void resetEncoders() {
		leftEncoder.setPosition(0.0);
		rightEncoder.setPosition(0.0);
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
