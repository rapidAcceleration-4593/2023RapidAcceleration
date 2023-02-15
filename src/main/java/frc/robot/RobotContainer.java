package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auton.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.armCommands.armExtensionIn;
import frc.robot.commands.armCommands.armExtensionOut;
import frc.robot.commands.armCommands.armStop;
import frc.robot.commands.armCommands.moveDownPoint;
import frc.robot.commands.armCommands.moveLeft;
import frc.robot.commands.armCommands.moveRight;
import frc.robot.commands.armCommands.moveUpPoint;
import frc.robot.commands.armCommands.theReverseScorer;
import frc.robot.commands.armCommands.theScorer;
import frc.robot.commands.armCommands.manualArmControl.armBaseRotateLeft;
import frc.robot.commands.armCommands.manualArmControl.armBaseRotateRight;
import frc.robot.commands.armCommands.manualArmControl.armDown;
import frc.robot.commands.armCommands.manualArmControl.armUp;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.vision;

public class RobotContainer {
  // Auton Chooser
  private final SendableChooser<Command> m_autonChooser = new SendableChooser<Command>();

  // Robot subsystems
  private Swerve m_swerve = new Swerve();
  private arm m_arm = new arm();
  private vision m_Vision = new vision();

  // Xbox controllers
  public static final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.xboxController1Port);
  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public RobotContainer() {
    configureButtonBindings();
    configureAxisActions();
    registerAutons();
  }

  /** Used for defining button actions. */
  private void configureButtonBindings() {

    /* Driver Buttons */
    driver.x().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));

    driver.a().whileTrue(new moveDownPoint(m_arm));
    driver.a().whileFalse(new armStop(m_arm));
    
    driver.y().whileTrue(new moveUpPoint(m_arm));
    driver.y().whileFalse(new armStop(m_arm));

    driver.leftBumper().whileTrue(new moveLeft(m_arm));
    driver.leftBumper().whileFalse(new armStop(m_arm));

    driver.rightBumper().whileTrue(new moveRight(m_arm));
    driver.rightBumper().whileFalse(new armStop(m_arm));

    driver.rightTrigger().whileTrue(new theScorer(m_arm));
    driver.rightTrigger().whileFalse(new armStop(m_arm));

    driver.leftTrigger().whileTrue(new theReverseScorer(m_arm));
    driver.leftTrigger().whileFalse(new armStop(m_arm));

    driver.start().onTrue(new armExtensionOut(m_arm));
    driver.start().onFalse(new armStop(m_arm));

    driver.back().onTrue(new armExtensionIn(m_arm));
    driver.back().onFalse(new armStop(m_arm));

  }

  /** Used for joystick/xbox axis actions. */
  private void configureAxisActions() {
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> driver.rightStick().getAsBoolean()));
  }

  /** Register the autonomous modes to the chooser for the drivers to select. */
  public void registerAutons() {
    // Register autons.
    m_autonChooser.setDefaultOption("No-op", new InstantCommand());

    // Push the chooser to the dashboard.
    SmartDashboard.putData("Auton Chooser", m_autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_autonChooser.getSelected();
    return new Autos(m_swerve);
  }
}
