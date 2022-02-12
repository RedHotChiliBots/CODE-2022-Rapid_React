// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CLIMB;
import frc.robot.commands.ChassisArcadeDrive;
import frc.robot.commands.ChassisTankDrive;
import frc.robot.commands.ClimberGoTo;
import frc.robot.commands.ClimberHighTravClimb;
import frc.robot.commands.ClimberMidRungClimb;
import frc.robot.commands.ClimberPerpendicular;
import frc.robot.commands.ClimberSwivel;
import frc.robot.commands.IntakeArmExtend;
import frc.robot.commands.IntakeArmRetract;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.DriveForward;
import frc.robot.commands.SHOOT;
import frc.robot.commands.ShooterPlungerExtend;
import frc.robot.commands.ShooterPlungerRetract;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final Chassis chassis = new Chassis();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter();

  // =============================================================
  // Define Joysticks
  private final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

  private static final double DEADZONE = 0.3;

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
      () -> getJoystick(driver.getLeftY()), () -> getJoystick(driver.getRightY()));

  private final ShooterShoot shoot = new ShooterShoot(shooter);
  private final ShooterStop shooterStop = new ShooterStop(shooter);
  private final ShooterPlungerExtend plungerExtend = new ShooterPlungerExtend(shooter);
  private final ShooterPlungerRetract plungerRetract = new ShooterPlungerRetract(shooter);
  private final SHOOT SHOOT = new SHOOT(shooter);

  private final ClimberSwivel swivel = new ClimberSwivel(climber);
  private final ClimberPerpendicular perpendicular = new ClimberPerpendicular(climber);
  private final ClimberGoTo toClearMidRung = new ClimberGoTo(climber, ClimberConstants.kClearLowRung);
  private final ClimberGoTo toMidRung = new ClimberGoTo(climber, ClimberConstants.kLowRung);
  private final ClimberGoTo toFullExtendPerp = new ClimberGoTo(climber, ClimberConstants.kFullExtendPerpendicular);
  private final ClimberGoTo toFullExtendSwivel = new ClimberGoTo(climber, ClimberConstants.kFullExtendSwivel);
  private final ClimberGoTo toStow = new ClimberGoTo(climber, ClimberConstants.kStow);
  private final ClimberMidRungClimb midRungClimb = new ClimberMidRungClimb(climber, chassis);
  private final ClimberHighTravClimb highTravClimb = new ClimberHighTravClimb(climber);
  private final CLIMB CLIMB = new CLIMB(climber, chassis);

  private final IntakeArmExtend intakeArmExtend = new IntakeArmExtend(intake);
  private final IntakeArmRetract intakeArmRetract = new IntakeArmRetract(intake);
  private final IntakeStop intakeStop = new IntakeStop(intake);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // ==============================================================================
    // Add Subsystems to Dashboard
    SmartDashboard.putData("Chassis", chassis);
    SmartDashboard.putData("Shooter", shooter);
    SmartDashboard.putData("Climber", climber);
    SmartDashboard.putData("Intake", intake);
    SmartDashboard.putData("Hopper", hopper);

    // ==============================================================================
    // Add commands to the autonomous command chooser
    chooser.setDefaultOption("Tank Drive", chassisTankDrive);
    chooser.addOption("Arcade Drive", chassisArcadeDrive);
    // Put the chooser on the dashboard
    SmartDashboard.putData(chooser);

    // =============================================================
    // Configure default commands for each subsystem
    chassis.setDefaultCommand(chassisTankDrive);
    //climber.setDefaultCommand(climberStop);
    intake.setDefaultCommand(intakeStop);
    //hopper.setDefaultCommand(hopperStop);
    shooter.setDefaultCommand(shooterStop);
  }

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
    new JoystickButton(driver, Button.kX.value).whenPressed(modifiedTankDrive);

    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(shoot);
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(shooterStop);

    new JoystickButton(operator, Button.kStart.value).whenPressed(swivel);
    new JoystickButton(operator, Button.kBack.value).whenPressed(perpendicular);

    new JoystickButton(operator, Button.kY.value).whenPressed(toClearMidRung);
    new JoystickButton(operator, Button.kB.value).whenPressed(toMidRung);
    new JoystickButton(operator, Button.kX.value).whenPressed(toFullExtendPerp);
    new JoystickButton(operator, Button.kA.value).whenPressed(toFullExtendSwivel);
    // Will need a stow at soem point but will add in when rest is auto command
    // because not enough buttons for testing
    // new JoystickButton(operator, Button..value).whenPressed(toStow);
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
