// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_l1;
  private CANSparkMax m_r1;
  private SlewRateLimiter m_limiter;

  public Shooter() {
    m_l1 = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushed);
    m_r1 = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushed);
    m_r1.follow(m_l1, true);
    m_limiter = new SlewRateLimiter(4);
  }

  public void setMotors(double powerPercent) {
    if (powerPercent != 0) {
      m_l1.set(m_limiter.calculate(powerPercent));
    } else {
      m_l1.set(0.0);
    }
  }
}