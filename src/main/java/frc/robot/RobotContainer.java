// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Drivetrain m_drivetrain;
  Elevator m_Elevator;
  Shooter m_Shooter;
  Intake m_Intake;
  LEDs m_LEDs;

  CommandJoystick m_driverJoystick;
  CommandJoystick m_operatorJoystick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driverJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
    m_operatorJoystick = new CommandJoystick(OperatorConstants.kOperatorControllerPort);
    // Configure the trigger bindings
    configureBindings();
    /* UsbCamera _outCamera = */ CameraServer.startAutomaticCapture(0);

    m_drivetrain.setDefaultCommand(new DriveCommand(m_drivetrain, () -> {
      return m_driverJoystick.getX();
    }, () -> {
      return m_driverJoystick.getY();
    }, () -> {
      return m_driverJoystick.getZ();
    }, () -> {
      return m_driverJoystick.getThrottle() < 0.25;
    }));

    m_LEDs.setDefaultCommand(new LEDChaseCommand(m_LEDs, 255, 50, 0, 255, 255, 255, 40));
  }

  private void configureBindings() {
    // driver setup
    m_driverJoystick.setZChannel(4);
    m_driverJoystick.button(5).onTrue(m_drivetrain.resetGyro());

    // operator setup
    m_operatorJoystick.povUp().whileTrue(new ElevatorManualCommand(m_Elevator, ElevatorConstants.speed));
    m_operatorJoystick.povDown().whileTrue(new ElevatorManualCommand(m_Elevator, -ElevatorConstants.speed));

    m_operatorJoystick.button(OperatorConstants.elevatorUpButton).onTrue(new ElevatorToTopCommand(m_Elevator));
    m_operatorJoystick.button(OperatorConstants.elevatorDownButton).onTrue(new ElevatorToBottomCommand(m_Elevator));

    m_operatorJoystick.button(OperatorConstants.intakeButton).whileTrue(new RunIntakeCommand(m_Intake, 0.1));
    m_operatorJoystick.button(OperatorConstants.intakeReverseButton).whileTrue(new RunIntakeCommand(m_Intake, -0.1));

    m_operatorJoystick.axisGreaterThan(OperatorConstants.moveToShooterAxis, 0.15)
        .whileTrue(new ShootCommand(m_Shooter, 1.0));
    m_operatorJoystick.axisGreaterThan(OperatorConstants.moveToShooterAxis, 0.85)
        .whileTrue(new RunIntakeCommand(m_Intake, 1.0));

    // field commands
    new Trigger(DriverStation::isEnabled)
        .onTrue(new LEDChaseCommand(m_LEDs, 255, 255, 255,
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 255 : 0, 0,
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 255 : 0, 40));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ElevatorToTopCommand(m_Elevator).withTimeout(1)
        .andThen(new WaitCommand(1))
        .andThen(new RunIntakeCommand(m_Intake, -0.1)).withTimeout(1)
        .andThen(new WaitCommand(1))
        .andThen(
            new ShootCommand(m_Shooter, 1.0).andThen(new WaitCommand(1).andThen(new RunIntakeCommand(m_Intake, 1.0))))
        .andThen(new DriveDistanceCommand(m_drivetrain, 0.0, -0.381)).withTimeout(1);
  }
}
