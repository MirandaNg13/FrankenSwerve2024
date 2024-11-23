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

    public static final double kTurningEncoderPositionFactor = kWheelDiameterMeters/(gearRatio1st*gearRatio2nd*gearRatio3rd);
    public static final double kTurningEncoderVelocityFactor = kDrivingEncoderPositionFactor/60; // meters per second
    
    public static final double driveGainP = 1;
    public static final double driveGainI = 0;
    public static final double driveGainD = 0;

    public static final double turnGainP = 0.3;
    public static final double turnGainI = 0;
    public static final double turnGainD = 0;

    public static final double kAngleEncoderResolution = 4096;

    
  }

  public static class DriveConstants {
    public static final double robotWidth = Units.inchesToMeters(37);
    public static final double robothLength = Units.inchesToMeters(37);
    public static final double WheelYdist = robotLength*0.5;
    public static final double WheelXdist = robotWidth*0.5;
  }

public static class ControlSystem {
    // Driving motor CAN IDs
    public static final int kLeftFrontDrive = 2;
    public static final int kLeftBackDrive = 4;
    public static final int kRightFrontDrive = 1;
    public static final int kRightBackDrive = 3;
    // Turning motors
    public static final int kLeftFrontTurn = 10;
    public static final int kLeftBackTurn = 14;
    public static final int kRightFrontTurn = 13;
    public static final int kRightBackTurn = 15;
// no idea if I should swap the A and Bs.
    public static final int kLFturnA = 2;
    public static final int kLFturnB = 3;
    public static final int kLBturnA = 0;
    public static final int kLBturnB = 1;
    public static final int kRFturnA = 6;
    public static final int kRFturnB = 7;
    public static final int kRBturnA = 4;
    public static final int kRBturnB = 5;
  }
}
