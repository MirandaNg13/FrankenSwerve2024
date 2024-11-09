// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double gearRatio1st = 14/50;
    public static final double gearRatio2nd = 27/17;
    public static final double gearRatio3rd = 15/45;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // meters

    public static final double kDrivingEncoderPositionFactor = kWheelDiameterMeters/(gearRatio1st*gearRatio2nd*gearRatio3rd);
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor/60; // meters per second
    
    public static final double driveGainP = 1;
    public static final double driveGainI = 0;
    public static final double driveGainD = 0;

    public static final double turnGainP = 0.3;
    public static final double turnGainI = 0;
    public static final double turnGainD = 0;

    public static final double kAngleEncoderResolution = 4096;

    
  }

  public static class ControlSystem {
  
  }
}
