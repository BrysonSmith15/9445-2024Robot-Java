// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;

import frc.robot.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_flModule = new SwerveModule(14, 23, 13, true, true);
    m_frModule = new SwerveModule(11, 3, 10, false, true);
    m_flModule = new SwerveModule(17, 15, 16, true, true);
    m_flModule = new SwerveModule(8, 6, 7, false, true);

    double offset = 0.381;
    m_kinematics = new SwerveDriveKinematics(new Translation2d(offset, offset), new Translation2d(offset, -offset),
        new Translation2d(-offset, offset), new Translation2d(-offset, -offset));
    try {
      m_gyro = new AHRS(SerialPort.Port.kUSB);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error getting gyro" + ex.getMessage(), true);
    }
    m_odometry = new SwerveDriveOdometry(
        m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] { m_flModule.getPosition(),
            m_frModule.getPosition(), m_blModule.getPosition(), m_brModule.getPosition() },
        new Pose2d(0, 0, new Rotation2d(0)));
  }

  // Degrees -180 -> 180
  public double getGyroAngle() {
    return m_gyro.getYaw();
  }

  public Rotation2d getGyroRot2d() {
    return m_gyro.getRotation2d();
  }

  public void setStates(SwerveModuleState[] states) {
    m_flModule.setDesiredState(states[0]);
    m_frModule.setDesiredState(states[1]);
    m_blModule.setDesiredState(states[2]);
    m_brModule.setDesiredState(states[3]);
  }

  public void stop() {
    SwerveModuleState stopState = new SwerveModuleState(0, new Rotation2d(0));
    m_flModule.setDesiredState(stopState);
    m_frModule.setDesiredState(stopState);
    m_blModule.setDesiredState(stopState);
    m_brModule.setDesiredState(stopState);
  }

  public Command resetGyro() {
    return this.runOnce(() -> m_gyro.reset());
  }

  public Command setIdleState(boolean coast) {
    return this.runOnce(() -> {
      m_flModule.setIdleState(coast);
      m_frModule.setIdleState(coast);
      m_blModule.setIdleState(coast);
      m_brModule.setIdleState(coast);
    });
  }

  // meters
  public double getDistance() {
    return m_flModule.getPosition().distanceMeters;
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[] { m_flModule.getPosition(),
        m_frModule.getPosition(), m_blModule.getPosition(), m_brModule.getPosition() });
  }

  public SwerveDriveKinematics m_kinematics;
  public SwerveDriveOdometry m_odometry;
  private AHRS m_gyro;
  private SwerveModule m_flModule;
  private SwerveModule m_frModule;
  private SwerveModule m_blModule;
  private SwerveModule m_brModule;
}
