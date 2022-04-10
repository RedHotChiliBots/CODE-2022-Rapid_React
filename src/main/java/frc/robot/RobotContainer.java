// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber.LatchState;
import frc.robot.subsystems.Climber.SwivelState;
import frc.robot.subsystems.Collector.ArmState;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DoRumble;
import frc.robot.commands.DrivePosition;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.FeederRun;
import frc.robot.commands.HopperShoot;
import frc.robot.commands.FeederStop;
import frc.robot.commands.FeederSuckIn;
import frc.robot.commands.HopperRun;
import frc.robot.commands.HopperStop;
import frc.robot.commands.REDAUTONSHOOTTOTERM;
import frc.robot.commands.REDFOURCARGOAUTON;
import frc.robot.commands.REDNOTCENTERRUNGSIDETOTERM;
import frc.robot.commands.REDONECARGOAUTON;
import frc.robot.commands.REDONECARGOMIDAUTON;
import frc.robot.commands.AUTONFOURORTENCARGOOUTANDBACK;
import frc.robot.commands.AUTONONEORSEVENCARGOOUTANDBACK;
import frc.robot.commands.AUTONTAXI;
import frc.robot.commands.AUTONTWOOREIGHTCARGOOUTANDBACK;
import frc.robot.commands.BLUEAUTONSHOOTTOTERM;
import frc.robot.commands.BLUEFOURCARGOAUTON;
import frc.robot.commands.BLUENOTCENTERTOTERM;
import frc.robot.commands.BLUEONECARGOAUTON;
import frc.robot.commands.BLUEONECARGOMIDAUTON;
import frc.robot.commands.ChassisArcadeDrive;
import frc.robot.commands.ChassisTankDrive;
import frc.robot.commands.ClimbCancel;
import frc.robot.commands.ClimberSwivelAndEngageHighTrav;
import frc.robot.commands.ClimberTravClimb;
import frc.robot.commands.ClimberUnlatchAndPullUp;
import frc.robot.commands.CollectorArm;
import frc.robot.commands.CollectorRun;
import frc.robot.commands.CollectorStop;
// import frc.robot.commands.SHOOT;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.ShooterSuckIn;
import frc.robot.commands.TAKEIN;
import frc.robot.commands.CLIMB;
import frc.robot.commands.COLLECT;
import frc.robot.commands.COLLECTORSTOWSTOP;
import frc.robot.commands.ChassisMonitorPitch;
import frc.robot.commands.ClimberGoTo;
import frc.robot.commands.ClimberHighClimb;
import frc.robot.commands.ClimberInit;
import frc.robot.commands.ClimberLatch;
import frc.robot.commands.ClimberLatchAndReadyForNext;
import frc.robot.commands.ClimberLeftInit;
import frc.robot.commands.ClimberMidRungClimb;
import frc.robot.commands.ClimberPerpAndHookHighTrav;
import frc.robot.commands.ClimberSetup;
import frc.robot.commands.ClimberSwivel;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private static final Chassis chassis = new Chassis();
	private static final Climber climber = new Climber(chassis);
	private final Collector collector = new Collector();
	private final Hopper hopper = new Hopper();
	// private final Feeder feeder = new Feeder();
	private final Shooter shooter = new Shooter();

	// =============================================================
	// Define Joysticks
	public final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);
	public final XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

	// Define a chooser for autonomous commands
	private final SendableChooser<Command> chooser = new SendableChooser<>();

	// =============================================================
	// Define Commands here to avoid multiple instantiations
	// If commands use Shuffleboard and are instantiated multiple time, an error
	// is thrown on the second instantiation becuase the "title" already exists.
	private final ChassisTankDrive chassisTankDrive = new ChassisTankDrive(chassis,
			() -> getJoystick(driver.getLeftY()), () -> getJoystick(driver.getRightY()));
	private final ChassisTankDrive modifiedTankDrive = new ChassisTankDrive(chassis,
			() -> -0.4, () -> -0.4);
	private final ChassisArcadeDrive chassisArcadeDrive = new ChassisArcadeDrive(chassis,
			() -> getJoystick(driver.getLeftY()), () -> getJoystick(driver.getRightX()));

	private final HopperRun hopperRun = new HopperRun(hopper, collector);
	private final HopperStop hopperStop = new HopperStop(hopper);

	// private final FeederRun feederRun = new FeederRun(feeder, shooter);
	// private final FeederStop feederStop = new FeederStop(feeder);

	private final ShooterRun shooterRun = new ShooterRun(shooter, hopper);
	private final ShooterStop shooterStop = new ShooterStop(shooter);

	// private final SHOOT SHOOT = new SHOOT(shooter, hopper);
	private final ShooterShoot shooterShoot = new ShooterShoot(shooter, hopper);

	private final CLIMB climb = new CLIMB(climber, collector, chassis);
	private final ClimberInit climberInit = new ClimberInit(climber);
	private final ClimberGoTo toStow = new ClimberGoTo(climber, ClimberConstants.kStow);
	private final ClimberSetup climbSetup = new ClimberSetup(climber, collector);
	private final ClimberGoTo toMidRungEngage = new ClimberGoTo(climber, ClimberConstants.kEngageMidRung);
	private final ClimberGoTo toHighTravEngage = new ClimberGoTo(climber, ClimberConstants.kEngageHighTrav);
	private final ClimberGoTo toHighTravLatch = new ClimberGoTo(climber, ClimberConstants.kHookHighTrav);
	private final ClimberMidRungClimb climbMidRung = new ClimberMidRungClimb(climber);
	private final ClimberHighClimb climbHighRung = new ClimberHighClimb(climber, chassis);
	private final ClimberTravClimb climbTravRung = new ClimberTravClimb(climber, collector, chassis);

	private final ChassisMonitorPitch chassisMonitorPitch = new ChassisMonitorPitch(chassis);

	private final ClimberGoTo extend = new ClimberGoTo(climber, ClimberConstants.kEngageHighTrav);
	private final ClimberGoTo retract = new ClimberGoTo(climber, ClimberConstants.kStow);
	private final ClimberSwivel swivel = new ClimberSwivel(climber, SwivelState.SWIVEL);
	private final ClimberSwivel perpendicular = new ClimberSwivel(climber, SwivelState.PERPENDICULAR);
	private final ClimberLatch climberOpen = new ClimberLatch(climber, LatchState.OPEN);
	private final ClimberLatch climberClose = new ClimberLatch(climber, LatchState.CLOSE);

	private final ClimberGoTo toClearMidRung = new ClimberGoTo(climber, ClimberConstants.kClearLowRung);
	private final ClimberGoTo toMidRung = new ClimberGoTo(climber, ClimberConstants.kLowRung);
	private final ClimberGoTo toOneRev = new ClimberGoTo(climber, ClimberConstants.kOneRev);

	private final CollectorArm collectorDeploy = new CollectorArm(collector, ArmState.DEPLOY);
	private final CollectorArm collectorStow = new CollectorArm(collector, ArmState.STOW);
	private final CollectorStop collectorStop = new CollectorStop(collector);
	private final COLLECTORSTOWSTOP collectorStowStop = new COLLECTORSTOWSTOP(collector);
	private final CollectorRun collectorCollect = new CollectorRun(collector);
	private final COLLECT collect = new COLLECT(collector, hopper);

	private final DoRumble doRumble = new DoRumble(this);

	private String BlueRungSideCargoToHubJSON = "paths/output/BlueRungSideCargoToHub.wpilib.json";
	public static Trajectory BlueRungSideCargoToHub = null;
	// private String BlueRungSideHubToCargoJSON =
	// "paths/output/BlueRungSideHubToCargo.wpilib.json";
	// public static Trajectory BlueRungSideHubToCargo = null;
	// private String unnamedJSON = "paths/output/Unnamed.wpilib.json";
	// public static Trajectory unnamed = null;

	private String RedTermSideOneCargoJSON = "paths/output/A1A.wpilib.json";
	public static Trajectory RedTermSideOneCargo = null;
	private String RedTermSideCargoAndTermJSON = "paths/output/A2TermA.wpilib.json";
	public static Trajectory RedTermSideCargoAndTerm = null;
	private String BlueTermSideOneCargoJSON = "paths/output/C7C.wpilib.json";
	public static Trajectory BlueTermSideOneCargo = null;
	private String BlueTermSideCargoAndTermJSON = "paths/output/C8TermC.wpilib.json";
	public static Trajectory BlueTermSideCargoAndTerm = null;
	private String RedRungSideMidJSON = "paths/output/G4B.wpilib.json";
	public static Trajectory RedRungSideMid = null;
	private String BlueRungSideMidJSON = "paths/output/L10D.wpilib.json";
	public static Trajectory BlueRungSideMid = null;
	private String BlueNotCenterRungSideToTermJSON = "paths/output/BlueNotCenterRungSideToTerm.wpilib.json";
	public static Trajectory BlueNotCenterRungSideToTerm = null;
	private String RedNotCenterRungSideToTermJSON = "paths/output/RedNotCenterRungSideToTerm.wpilib.json";
	public static Trajectory RedNotCenterRungSideToTerm = null;

	public final REDONECARGOAUTON redOneCargoAuton;
	public final REDONECARGOMIDAUTON redOneCargoMidAuton;
	public final REDFOURCARGOAUTON redFourCargoAuton;
	public final BLUEONECARGOAUTON blueOneCargoAuton;
	public final BLUEFOURCARGOAUTON blueFourCargoAuton;
	public final BLUEONECARGOMIDAUTON blueOneCargoMidAuton;
	public final BLUEAUTONSHOOTTOTERM blueAutonShootToTerm;
	public final REDAUTONSHOOTTOTERM redAutonShootToTerm;
	public final BLUENOTCENTERTOTERM blueNotCenterToTerm;
	public final REDNOTCENTERRUNGSIDETOTERM redNotCenterToTerm;
	public final AUTONFOURORTENCARGOOUTANDBACK fourOrTenOutAndBack = new AUTONFOURORTENCARGOOUTANDBACK(chassis, collector,
			hopper, shooter);
	public final AUTONONEORSEVENCARGOOUTANDBACK oneOrSevenOutAndBack = new AUTONONEORSEVENCARGOOUTANDBACK(chassis,
			collector, hopper, shooter);
	public final AUTONTWOOREIGHTCARGOOUTANDBACK twoOrEightOutAndBack = new AUTONTWOOREIGHTCARGOOUTANDBACK(chassis,
			collector, hopper, shooter);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// ==============================================================================
		// Add Subsystems to Dashboard
		SmartDashboard.putData("Chassis", chassis);
		SmartDashboard.putData("Shooter", shooter);
		SmartDashboard.putData("Climber", climber);
		SmartDashboard.putData("Collector", collector);
		SmartDashboard.putData("Hopper", hopper);
		// SmartDashboard.putData("Feeder", feeder);

		// =============================================================
		// Configure default commands for each subsystem
		chassis.setDefaultCommand(chassisArcadeDrive);
		// chassis.setDefaultCommand(chassisTankDrive);
		// climber.setDefaultCommand(climberStop);
		collector.setDefaultCommand(collectorStop);
		hopper.setDefaultCommand(hopperStop);
		shooter.setDefaultCommand(shooterStop);

		try {
			// Path BlueRungSideCargoToHubPath = Filesystem.getDeployDirectory().toPath()
			// .resolve(BlueRungSideCargoToHubJSON);

			Path RedTermSideOneCargoPath = Filesystem.getDeployDirectory().toPath()
					.resolve(RedTermSideOneCargoJSON);
			RedTermSideOneCargo = TrajectoryUtil.fromPathweaverJson(RedTermSideOneCargoPath);
			Path RedTermSideCargoTermPath = Filesystem.getDeployDirectory().toPath()
					.resolve(RedTermSideCargoAndTermJSON);
			RedTermSideCargoAndTerm = TrajectoryUtil.fromPathweaverJson(RedTermSideCargoTermPath);
			Path BlueTermSideOneCargoPath = Filesystem.getDeployDirectory().toPath()
					.resolve(BlueTermSideOneCargoJSON);
			BlueTermSideOneCargo = TrajectoryUtil.fromPathweaverJson(BlueTermSideOneCargoPath);
			Path BlueTermSideCargoTermPath = Filesystem.getDeployDirectory().toPath()
					.resolve(BlueTermSideCargoAndTermJSON);
			BlueTermSideCargoAndTerm = TrajectoryUtil.fromPathweaverJson(BlueTermSideCargoTermPath);
			Path RedRungSideMidPath = Filesystem.getDeployDirectory().toPath()
					.resolve(RedRungSideMidJSON);
			RedRungSideMid = TrajectoryUtil.fromPathweaverJson(RedRungSideMidPath);
			Path BlueRungSideMidPath = Filesystem.getDeployDirectory().toPath()
					.resolve(BlueRungSideMidJSON);
			BlueRungSideMid = TrajectoryUtil.fromPathweaverJson(BlueRungSideMidPath);
			Path BlueNotCenterRungSideToTermPath = Filesystem.getDeployDirectory().toPath()
					.resolve(BlueNotCenterRungSideToTermJSON);
			BlueNotCenterRungSideToTerm = TrajectoryUtil.fromPathweaverJson(BlueNotCenterRungSideToTermPath);
			Path RedNotCenterRungSideToTermPath = Filesystem.getDeployDirectory().toPath()
					.resolve(RedNotCenterRungSideToTermJSON);
			RedNotCenterRungSideToTerm = TrajectoryUtil.fromPathweaverJson(RedNotCenterRungSideToTermPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + BlueRungSideCargoToHubJSON, ex.getStackTrace());
		}

		redOneCargoAuton = new REDONECARGOAUTON(chassis, collector, hopper, shooter);
		redOneCargoMidAuton = new REDONECARGOMIDAUTON(chassis, collector, hopper, shooter);
		redFourCargoAuton = new REDFOURCARGOAUTON(chassis, collector, hopper, shooter);
		blueOneCargoAuton = new BLUEONECARGOAUTON(chassis, collector, hopper, shooter);
		blueFourCargoAuton = new BLUEFOURCARGOAUTON(chassis, collector, hopper, shooter);
		blueOneCargoMidAuton = new BLUEONECARGOMIDAUTON(chassis, collector, hopper, shooter);
		blueAutonShootToTerm = new BLUEAUTONSHOOTTOTERM(chassis, hopper, shooter);
		redAutonShootToTerm = new REDAUTONSHOOTTOTERM(chassis, hopper, shooter);
		blueNotCenterToTerm = new BLUENOTCENTERTOTERM(chassis);
		redNotCenterToTerm = new REDNOTCENTERRUNGSIDETOTERM(chassis);

		// ==============================================================================
		// Add commands to the autonomous command chooser
		chooser.setDefaultOption("Tank Drive", chassisTankDrive);
		// chooser.addOption("Arcade Drive", chassisArcadeDrive);
		// chooser.addOption("Modified Tank Drive", modifiedTankDrive);

		// =============================================================
		// Build chooser for autonomous commands
		chooser.addOption("Out and Back Rung Side", fourOrTenOutAndBack);
		chooser.addOption("Out and Back by Wall", oneOrSevenOutAndBack);
		chooser.addOption("Out and Back by Term", twoOrEightOutAndBack);
		chooser.addOption("One Cargo", redOneCargoAuton);
		// chooser.addOption("Blue One Cargo", blueOneCargoAuton);
		chooser.addOption("Four Cargo", redFourCargoAuton);
		// chooser.addOption("Blue Four Cargo", blueFourCargoAuton);
		chooser.addOption("One Cargo Not In Middle", redOneCargoMidAuton);
		// chooser.addOption("Blue One Cargo Not In Middle", blueOneCargoMidAuton);
		// chooser.addOption("Blue Shoot To Term", blueAutonShootToTerm);
		chooser.addOption("Shoot To Term", redAutonShootToTerm);
		// chooser.addOption("Blue Not Center To Term", blueNotCenterToTerm);
		// chooser.addOption("Red Not Center To Term", redNotCenterToTerm);
		chooser.addOption("Shoot", shooterShoot);
		chooser.addOption("Shoot and Taxi", new AUTONTAXI(chassis, shooter, hopper));
		chooser.addOption("Taxi", new DrivePosition(chassis, 3.0));
		//chooser.addOption("Spin", new DriveTrajectory(chassis, chassis.straight));

		// Put the chooser on the dashboard
		SmartDashboard.putData(chooser);

		configureButtonBindings();
	}

	// private final DriveTrajectory blueRungSideCargoToHubCommand = new
	// DriveTrajectory(chassis, BlueRungSideCargoToHub);

	//
	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// new JoystickButton(driver, Button.kA.value).whenPressed(chassisTankDrive);
		// new JoystickButton(driver, Button.kB.value).whenPressed(chassisArcadeDrive);

	//	new JoystickButton(driver, Button.kBack.value).whenPressed(new CollectorArm(collector, ArmState.STOW));
		new JoystickButton(driver, Button.kStart.value).whenPressed(swivel);
		new JoystickButton(driver, Button.kBack.value).whenPressed(perpendicular);
		new JoystickButton(driver, Button.kX.value).whenPressed(new ClimberInit(climber));
		new JoystickButton(driver, Button.kB.value).whenPressed(new HopperStop(hopper));
		new JoystickButton(driver, Button.kY.value).whenPressed(new ClimberGoTo(climber, ClimberConstants.kEngageHighTrav));
		new JoystickButton(driver, Button.kA.value).whenPressed(new ClimberGoTo(climber, ClimberConstants.kHookHighTrav));
		new JoystickButton(driver, Button.kRightBumper.value)
				.whenPressed(new ClimberGoTo(climber, ClimberConstants.kClearMidRung));

		new JoystickButton(operator, Button.kRightBumper.value).whenPressed(collect);
		new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(collectorStowStop);
		new JoystickButton(operator, Button.kStart.value).whenPressed(shooterShoot);
		new JoystickButton(operator, Button.kBack.value).whenPressed(new ClimbCancel(climber));
		new JoystickButton(operator, Button.kY.value).whenPressed(climbSetup);
		new JoystickButton(operator, Button.kB.value).whenPressed(climb);
		new JoystickButton(operator, Button.kA.value).whenPressed(climberOpen);
		new JoystickButton(operator, Button.kX.value).whenPressed(climberClose);
		// new JoystickButton(operator, Button.kX.value).whenPressed(climbMidRung);

		// new JoystickButton(operator, Button.kA.value).whenPressed(climbHighRung);
		// new JoystickButton(operator, Button.kB.value).whenPressed(climbTravRung);

		// new JoystickButton(operator, Button.kA.value).whenPressed(extend);
		// new JoystickButton(operator, Button.kB.value).whenPressed(retract);

		// new JoystickButton(operator, Button.kX.value).whenPressed(new SHOOT(shooter,
		// hopper));
		// new JoystickButton(operator, Button.kA.value).whenPressed(new TAKEIN(shooter,
		// hopper, collector));
		// new JoystickButton(operator, Button.kA.value).whenPressed(hopperRun);

		// new JoystickButton(operator, Button.kX.value).whenPressed(shooterRun);
		// new JoystickButton(operator, Button.kA.value).whenPressed(new
		// FeederShoot(feeder, hopper, shooter));

		// new JoystickButton(operator,
		// Button.kRightBumper.value).whenPressed(climbMidRung);
		// new JoystickButton(operator,
		// Button.kLeftBumper.value).whenPressed(climbHighRung);

		// new JoystickButton(operator, Button.kStart.value).whenPressed(new
		// DrivePosition(chassis, 1.0));
		// new JoystickButton(operator, Button.kBack.value).whenPressed(new
		// ClimbCancel(climber));

		// new JoystickButton(operator, Button.kStart.value).whenPressed(new
		// ShooterStop(shooter));
		// new JoystickButton(operator, Button.kBack.value).whenPressed(new
		// ClimberInit(climber));
		// new JoystickButton(driver, Button.kY.value).whenPressed(climbMidRung);
		// new JoystickButton(driver, Button.kB.value).whenPressed(climbHighRung);
		// new JoystickButton(driver, Button.kX.value).whenPressed(new
		// FeederStop(feeder));
		// new JoystickButton(driver, Button.kB.value).whenPressed(new HopperRun(hopper,
		// feeder, collector));

		// new JoystickButton(operator, Button.kY.value).whenPressed(doRumble);
		// new JoystickButton(operator, Button.kA.value).whenPressed(toOneRev);
		// new JoystickButton(operator, Button.kY.value).whenPressed(toClearMidRung);
		// new JoystickButton(operator, Button.kB.value).whenPressed(toMidRung);
		// new JoystickButton(operator, Button.kA.value).whenPressed(toHighTravEngage);
		// new JoystickButton(operator, Button.kB.value).whenPressed(toHighTravLatch);

		// new JoystickButton(driver, Button.kY.value).whenPressed(new
		// DriveTrajectory(chassis, BlueRungSideCargoToHub));
	}

	public static Climber getClimber() {
		return climber;
	}

	private static final double DEADZONE = 0.01;
	private static final double MAXACCEL = 0.001; // joystick units per 20ms
	private double lastValue = 0.0;

	public double getJoystick(double js) {
		double val = 0.0;
		if (js > 0) {
			val = Math.pow(Math.abs(js) < DEADZONE ? 0.0 : js, 2);
		} else {
			val = -1 * Math.pow(Math.abs(js) < DEADZONE ? 0.0 : js, 2);
		}

		// if (Math.abs(val - lastValue) > MAXACCEL) {
		// lastValue += MAXACCEL;
		// val = lastValue;
		// }

		return val;
	}

	public void setDriverRumble(GenericHID.RumbleType t) {
		driver.setRumble(t, 1);
	}

	public void resetDriverRumble(GenericHID.RumbleType t) {
		driver.setRumble(t, 0);
	}

	public void setOperatorRumble(GenericHID.RumbleType t) {
		operator.setRumble(t, 1);
	}

	public void resetOperatorRumble(GenericHID.RumbleType t) {
		operator.setRumble(t, 0);
	}

	public void doRumble(XboxController c, GenericHID.RumbleType t) {
		Thread thread = new Thread("Rumble") {
			@Override
			public void run() {

				c.setRumble(t, 1);

				try {
					TimeUnit.MILLISECONDS.sleep(OIConstants.kRumbleDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("Rumble sleep exception", true);
				}

				c.setRumble(t, 0);
			}
		};
		thread.start();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
