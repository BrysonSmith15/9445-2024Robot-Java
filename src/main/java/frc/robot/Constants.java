// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int moveToShooterAxis = 3;

    public static final int intakeButton = 1;
    public static final int intakeReverseButton = 2;
    public static final int elevatorManualDownAngle = 180;
    public static final int elevatorManualUpAngle = 0;

    public static final int elevatorDownButton = 7;
    public static final int elevatorUpButton = 8;
  }

  public static class DrivetrainConstants {
    public static final double maxSpeedMPS = Units.feetToMeters(2.5);
    public static final double maxRotRadsPS = Units.degreesToRadians(45);
  }

  public static class ElevatorConstants {
    public static final int stableTicks = 650;
    public static final double speed = 0.45;
    public static final double stableUpSpeed = 0.1;
    public static final double stableDownSpeed = 0.0;
  }
}
