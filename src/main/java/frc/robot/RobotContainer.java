// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ChassisTankDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis m_chassis = new Chassis();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();

  // =============================================================
	// Define Joysticks
	XboxController m_driver = new XboxController(OIConstants.kDriverControllerPort);
	XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  private static final double DEADZONE = 0.3;

  // =============================================================
	// Define Commands here to avoid multiple instantiations
	// If commands use Shuffleboard and are instantiated multiple time, an error
	// is thrown on the second instantiation becuase the "title" already exists.
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_chassis);

  //Creating tabs on shuffleboard for each subsystem
  ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //==============================================================================
    //Add Subsystems to Dashboard
    SmartDashboard.putData("Chassis", m_chassis);
    SmartDashboard.putData("Shooter", m_shooter);
    SmartDashboard.putData("Climber", m_climber);

    // =============================================================
		// Configure default commands for each subsystem
    m_shooter.setDefaultCommand(new ShooterStop(m_shooter));
    m_chassis.setDefaultCommand(new ChassisTankDrive(m_chassis, () -> m_driver.getLeftY(), () -> m_driver.getRightY()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_operator, Button.kA.value).whenPressed(new ShooterShoot(m_shooter));
    new JoystickButton(m_operator, Button.kB.value).whenPressed(new ShooterStop(m_shooter));
    
  }

  public void setDriverRumble(GenericHID.RumbleType t) {
		m_driver.setRumble(t, 1);
	}

	public void resetDriverRumble(GenericHID.RumbleType t) {
		m_driver.setRumble(t, 0);
	}

	public void setOperatorRumble(GenericHID.RumbleType t) {
		m_operator.setRumble(t, 1);
	}

	public void resetOperatorRumble(GenericHID.RumbleType t) {
		m_operator.setRumble(t, 0);
	}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
