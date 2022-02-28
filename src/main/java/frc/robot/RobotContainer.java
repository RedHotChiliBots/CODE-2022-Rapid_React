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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber.LatchState;
import frc.robot.subsystems.Climber.SwivelState;
import frc.robot.subsystems.Collector.ArmState;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.CLIMB;
import frc.robot.commands.ChassisArcadeDrive;
import frc.robot.commands.ChassisTankDrive;
import frc.robot.commands.ClimberGoTo;
import frc.robot.commands.ClimberHighTravClimb;
import frc.robot.commands.ClimberInit;
import frc.robot.commands.ClimberLatch;
import frc.robot.commands.ClimberMidRungClimb;
import frc.robot.commands.ClimberSwivel;
import frc.robot.commands.DoRumble;
import frc.robot.commands.CollectorArm;
import frc.robot.commands.CollectorStop;
import frc.robot.commands.DrivePosition;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.SHOOT;
import frc.robot.commands.ShooterPlungerExtend;
import frc.robot.commands.ShooterPlungerRetract;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;

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
	private static final Climber climber = new Climber();
	private final Collector collector = new Collector();
	private final Hopper hopper = new Hopper();
	private final Shooter shooter = new Shooter();

	// =============================================================
	// Define Joysticks
	public final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);
	public final XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

	private static final double DEADZONE = 0.3;

	// Define a chooser for autonomous commands
	private final SendableChooser<Command> chooser = new SendableChooser<>();

	private Timer rumbleTimer = new Timer();

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

	private final ShooterShoot shoot = new ShooterShoot(shooter);
	private final ShooterStop shooterStop = new ShooterStop(shooter);
	private final SHOOT SHOOT = new SHOOT(shooter);

	private final ClimberSwivel swivel = new ClimberSwivel(climber, SwivelState.SWIVEL);
	private final ClimberSwivel perpendicular = new ClimberSwivel(climber, SwivelState.PERPENDICULAR);
	private final ClimberLatch climberOpen = new ClimberLatch(climber, LatchState.OPEN);
	private final ClimberLatch climberClose = new ClimberLatch(climber, LatchState.CLOSE);

	private final ClimberGoTo toClearMidRung = new ClimberGoTo(climber, ClimberConstants.kClearLowRung);
	private final ClimberGoTo toMidRung = new ClimberGoTo(climber, ClimberConstants.kLowRung);
	private final ClimberGoTo toOneRev = new ClimberGoTo(climber, ClimberConstants.kOneRev);
	private final ClimberInit climberInit = new ClimberInit(climber);
	private final ClimberGoTo toHighTravEngage = new ClimberGoTo(climber, ClimberConstants.kEngageHighTrav);
	private final ClimberGoTo toHighTravLatch = new ClimberGoTo(climber, ClimberConstants.kLatchHighTrav);
	private final ClimberGoTo toStow = new ClimberGoTo(climber, ClimberConstants.kStow);
	private final ClimberMidRungClimb midRungClimb = new ClimberMidRungClimb(climber, chassis);
	private final ClimberHighTravClimb highTravClimb = new ClimberHighTravClimb(climber);
	private final CLIMB CLIMB = new CLIMB(climber, chassis);

	private final CollectorArm collectorDeploy = new CollectorArm(collector, ArmState.DEPLOY);
	private final CollectorArm collectorStow = new CollectorArm(collector, ArmState.STOW);
	private final CollectorStop collectorStop = new CollectorStop(collector);

	private final DoRumble doRumble = new DoRumble(this);

	private String BlueRungSideCargoToHubJSON = "paths/output/BlueRungSideCargoToHub.wpilib.json";
	public static Trajectory BlueRungSideCargoToHub = null;
	// private String BlueRungSideHubToCargoJSON = "paths/output/BlueRungSideHubToCargo.wpilib.json";
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
	private String RedRungSideCrossJSON = "paths/output/H6B.wpilib.json";
	public static Trajectory RedRungSideCross = null;
	private String BlueRungSideCrossJSON = "paths/output/M12D.wpilib.json";
	public static Trajectory BlueRungSideCross = null;
	private String RedRungSideMidJSON = "paths/output/G4B.wpilib.json";
	public static Trajectory RedRungSideMid = null;
	private String BlueRungSideMidJSON = "paths/output/L10D.wpilib.json";
	public static Trajectory BlueRungSideMid = null;

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

		// ==============================================================================
		// Add commands to the autonomous command chooser
		chooser.setDefaultOption("Tank Drive", chassisTankDrive);
		chooser.addOption("Arcade Drive", chassisArcadeDrive);
		chooser.addOption("Modified Tank Drive", modifiedTankDrive);
		// Put the chooser on the dashboard
		SmartDashboard.putData(chooser);

		// =============================================================
		// Configure default commands for each subsystem
		chassis.setDefaultCommand(chassisTankDrive);
		// climber.setDefaultCommand(climberStop);
		collector.setDefaultCommand(collectorStop);
		// hopper.setDefaultCommand(hopperStop);
		shooter.setDefaultCommand(shooterStop);

		try {
			// Path BlueRungSideCargoToHubPath = Filesystem.getDeployDirectory().toPath()
			// 		.resolve(BlueRungSideCargoToHubJSON);
			
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
			Path RedRungSideCrossPath = Filesystem.getDeployDirectory().toPath()
					.resolve(RedRungSideCrossJSON);
			RedRungSideCross = TrajectoryUtil.fromPathweaverJson(RedRungSideCrossPath);
			Path BlueRungSideCrossPath = Filesystem.getDeployDirectory().toPath()
					.resolve(BlueRungSideCrossJSON);
			BlueRungSideCross = TrajectoryUtil.fromPathweaverJson(BlueRungSideCrossPath);
			Path RedRungSideMidPath = Filesystem.getDeployDirectory().toPath()
					.resolve(RedRungSideMidJSON);
			RedRungSideMid = TrajectoryUtil.fromPathweaverJson(RedRungSideMidPath);
			Path BlueRungSideMidPath = Filesystem.getDeployDirectory().toPath()
					.resolve(BlueRungSideMidJSON);
			BlueRungSideMid = TrajectoryUtil.fromPathweaverJson(BlueRungSideMidPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + BlueRungSideCargoToHubJSON, ex.getStackTrace());
		}

		rumbleTimer.reset();
		rumbleTimer.start();

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
		new JoystickButton(driver, Button.kA.value).whenPressed(chassisTankDrive);
		new JoystickButton(driver, Button.kB.value).whenPressed(chassisArcadeDrive);
		new JoystickButton(driver, Button.kX.value).whenPressed(new DrivePosition(chassis, 1.0));

		new JoystickButton(driver, Button.kStart.value).whenPressed(collectorDeploy);
		new JoystickButton(driver, Button.kBack.value).whenPressed(collectorStow);

		new JoystickButton(operator, Button.kRightBumper.value).whenPressed(climberOpen);
		new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(climberClose);

		new JoystickButton(operator, Button.kStart.value).whenPressed(swivel);
		new JoystickButton(operator, Button.kBack.value).whenPressed(perpendicular);

		new JoystickButton(operator, Button.kY.value).whenPressed(doRumble);
		new JoystickButton(operator, Button.kX.value).whenPressed(climberInit);
		new JoystickButton(operator, Button.kA.value).whenPressed(toOneRev);
		new JoystickButton(operator, Button.kB.value).whenPressed(toStow);

		// new JoystickButton(operator, Button..value).whenPressed(toStow);
		// 
		new JoystickButton(operator, Button.kY.value).whenPressed(toClearMidRung);
		// new JoystickButton(operator, Button.kB.value).whenPressed(toMidRung);
		// new JoystickButton(operator, Button.kA.value).whenPressed(toHighTravEngage);
		// new JoystickButton(operator, Button.kB.value).whenPressed(toHighTravLatch);

		// new JoystickButton(driver, Button.kY.value).whenPressed(new DriveTrajectory(chassis, BlueRungSideCargoToHub));
	}

	public static Climber getClimber() {
		return climber;
	}

	public double getJoystick(double js) {
		return Math.abs(js) < DEADZONE ? 0.0 : js;
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
