// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

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

  private final Drivetrain drivetrain = new Drivetrain();

  XboxController driveGamepad = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drivetrain.setDefaultCommand(new SwerveDrive(
        drivetrain,
        () -> (Constants.Drivetrain.SPEED_MULTIPLIER)
            * (driveGamepad.getLeftX() * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (Constants.Drivetrain.SPEED_MULTIPLIER)
            * -(driveGamepad.getLeftY() * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (Constants.Drivetrain.SPEED_MULTIPLIER)
            * (driveGamepad.getRightX() * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

    // Configure the button bindings
    configureButtonBindings();
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

    new JoystickButton(driveGamepad, 1)
        .whenPressed(() -> drivetrain.zeroGyroscope());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}