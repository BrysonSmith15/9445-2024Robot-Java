// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** An example command that uses an example subsystem. */
public class DriveDistanceCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private final double deadbandM = 0.381;

  private PIDController m_xPidController;
  private PIDController m_yPidController;

  // private double m_xDistM;
  // private double m_yDistM;

  /**
   * Creates a new DriveDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveDistanceCommand(Drivetrain drivetrain, double xDistM, double yDistM) {
    m_drivetrain = drivetrain;
    // m_xDistM = xDistM;
    // m_yDistM = yDistM;

    m_xPidController = new PIDController(6e-5, 0.0, 0.0);
    m_yPidController = new PIDController(6e-5, 0.0, 0.0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d current = m_drivetrain.m_odometry.getPoseMeters();
    startDistM = Math.sqrt(Math.pow(current.getX(), 2) + Math.pow(current.getX(), 2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d current = m_drivetrain.m_odometry.getPoseMeters();
    double dist = startDistM - Math.sqrt(Math.pow(current.getX(), 2) + Math.pow(current.getY(), 2));
    double xOut = m_xPidController.calculate(dist, 0);
    double yOut = m_yPidController.calculate(dist, 0);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xOut * DrivetrainConstants.maxSpeedMPS,
        yOut * DrivetrainConstants.maxSpeedMPS, 0.0,
        m_drivetrain.getGyroRot2d());

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
    Pose2d current = m_drivetrain.m_odometry.getPoseMeters();
    double dist = Math.abs(startDistM - Math.sqrt(Math.pow(current.getX(), 2) + Math.pow(current.getY(), 2)));
    return dist < deadbandM;
  }

  private double startDistM;
}
