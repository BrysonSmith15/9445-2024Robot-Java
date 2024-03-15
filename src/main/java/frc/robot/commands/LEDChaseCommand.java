// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example leds. */
public class LEDChaseCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LEDs m_leds;
  private int m_mr;
  private int m_mg;
  private int m_mb;
  private int m_cr;
  private int m_cg;
  private int m_cb;
  private int m_width;
  private int m_chasePoint;

  /**
   * Creates a new LEDChaseCommand.
   *
   * @param leds The leds used by this command.
   */
  public LEDChaseCommand(LEDs leds, int mr, int mg, int mb, int cr, int cg, int cb, int width) {
    m_leds = leds;
    // Use addRequirements() here to declare leds dependencies.
    addRequirements(leds);
    m_mr = mr;
    m_mg = mg;
    m_mb = mb;
    m_cr = cr;
    m_cb = cb;
    m_cb = cb;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chasePoint = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chasePoint++;
    m_chasePoint %= m_leds.stripLen;

    int chaseEnd = m_chasePoint + m_width;

    if (chaseEnd > m_leds.stripLen) {
      m_leds.set(m_chasePoint, chaseEnd, m_cr, m_cg, m_cb);
      m_leds.set(0, chaseEnd - m_leds.stripLen, m_cr, m_cg, m_cb);
      m_leds.set(chaseEnd - m_leds.stripLen + 1, m_chasePoint, m_mr, m_mg, m_mb);
    } else {
      m_leds.set(m_chasePoint, chaseEnd, m_cr, m_cg, m_cb);
      m_leds.set(0, m_chasePoint, m_mr, m_mg, m_mb);
      m_leds.set(chaseEnd, m_leds.stripLen, m_mr, m_mg, m_mb);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_leds.set(0, m_leds.stripLen, m_mr, m_mg, m_mb);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
