// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  public int stripLen;
  private AddressableLED led;
  private AddressableLEDBuffer ledBuf;

  /** Creates a new LEDs. */
  public LEDs() {
    led = new AddressableLED(9);
    stripLen = 80;
    led.setLength(stripLen);
    led.start();
  }

  public void set(int start, int end, int r, int g, int b) {
    for (int i = start; i < end; i++) {
      ledBuf.setRGB(i, r, g, b);
    }
  }

  public Command setCommand(int r, int g, int b) {
    return this.runOnce(() -> set(0, stripLen, r, g, b));
  }
}
