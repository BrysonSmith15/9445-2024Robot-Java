// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private CANSparkMax m_l1;
  private CANSparkMax m_l2;
  private CANSparkMax m_r1;
  private CANSparkMax m_r2;

  private DigitalInput m_botLimit;
  private DigitalInput m_topLimit;
  private Encoder m_encoder;
  private SlewRateLimiter m_limiter;

  public Elevator() {
    m_l1 = new CANSparkMax(26, CANSparkLowLevel.MotorType.kBrushed);
    m_l2 = new CANSparkMax(27, CANSparkLowLevel.MotorType.kBrushed);
    m_r1 = new CANSparkMax(24, CANSparkLowLevel.MotorType.kBrushed);
    m_r2 = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushed);

    m_botLimit = new DigitalInput(1);
    m_topLimit = new DigitalInput(0);
    m_encoder = new Encoder(8, 9);
    m_limiter = new SlewRateLimiter(2.0);
  }

  public boolean botPressed() {
    return m_botLimit.get();
  }

  public boolean topPressed() {
    return m_topLimit.get();
  }

  public void setMotors(double powerPercent) {
    powerPercent = -powerPercent;
    if (powerPercent < 0 && botPressed()) {
      powerPercent = 0;
    } else {
      powerPercent = m_limiter.calculate(powerPercent);
    }
    m_l1.set(powerPercent);
    m_l2.set(powerPercent);
    m_r1.set(powerPercent);
    m_r2.set(powerPercent);
  }

  public int getTicks() {
    return m_encoder.get();
  }

  @Override
  public void periodic() {
    if (botPressed()) {
      m_encoder.reset();
    }
  }
}
