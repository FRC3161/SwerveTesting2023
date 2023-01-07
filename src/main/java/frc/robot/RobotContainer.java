// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

  /* Controllers */
  private final XboxController driver = new XboxController(Constants.Operators.driver);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, Constants.Swerve.zeroGyro);
  private final JoystickButton robotCentric = new JoystickButton(driver, Constants.Swerve.robotCentric);
  private final JoystickButton perpendicular = new JoystickButton(driver, XboxController.Button.kY.value);
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(Constants.Swerve.translationAxis),
            () -> -driver.getRawAxis(Constants.Swerve.strafeAxis),
            () -> -driver.getRawAxis(Constants.Swerve.rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> driver.getPOV(),
            () -> driver.getRightBumper()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Actions that we want to do when the robot is disabled. */
  public void disabledActions() {
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
    /* Driver Buttons */
    zeroGyro.onTrue(
        new InstantCommand(() -> s_Swerve.zeroGyro()));
    perpendicular.onTrue(new PerpendicularTarget(s_Swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
