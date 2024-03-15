// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private DoubleSupplier m_xSupplier;
  private DoubleSupplier m_ySupplier;
  private DoubleSupplier m_thetaSupplier;
  private BooleanSupplier m_fieldOrientedSupplier;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier thetaSupplier, BooleanSupplier fieldOrientedSupplier) {
    m_drivetrain = drivetrain;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_thetaSupplier = thetaSupplier;
    m_fieldOrientedSupplier = fieldOrientedSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds;
    if (m_fieldOrientedSupplier.getAsBoolean()) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          m_xSupplier.getAsDouble() * DrivetrainConstants.maxSpeedMPS,
          m_ySupplier.getAsDouble() * DrivetrainConstants.maxSpeedMPS,
          m_thetaSupplier.getAsDouble() * DrivetrainConstants.maxRotRadsPS, m_drivetrain.getGyroRot2d());
    } else {
      speeds = new ChassisSpeeds(m_xSupplier.getAsDouble() * DrivetrainConstants.maxSpeedMPS,
          m_ySupplier.getAsDouble() * DrivetrainConstants.maxSpeedMPS,
          m_thetaSupplier.getAsDouble() * DrivetrainConstants.maxRotRadsPS);
    }
    SwerveModuleState[] states = m_drivetrain.m_kinematics.toSwerveModuleStates(speeds);

    m_drivetrain.setStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
