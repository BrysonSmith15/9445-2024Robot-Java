// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax m_motor;
  private SlewRateLimiter m_limiter;

  /** Creates a new Intake. */
  public Intake() {
    m_motor = new CANSparkMax(23, CANSparkLowLevel.MotorType.kBrushless);
    m_limiter = new SlewRateLimiter(.5);
  }

  public void setMotor(double powerPercent) {
    m_motor.set(m_limiter.calculate(powerPercent));
  }

  public void stop() {
    m_motor.set(0.0);
  }

}
