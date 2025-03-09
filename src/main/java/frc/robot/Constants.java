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
    public static final int ELEVATOR_MOTOR1_ID = 1;
    public static final int ELEVATOR_MOTOR2_ID = 2;
    public static final double elevatorPulleyDiameterInches = 2.0;
    public static final double elevatorGearRatio = 15.0;
    public static final double inchesPerMotorRotation = elevatorPulleyDiameterInches * Math.PI/elevatorGearRatio;
    public static final double kMaxLinearAccelInchesPerSSquared = 0;
    public static final double kMaxLinearRateInchesPerS = 0;
    public static final double kElevatorKd = 0;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKp = 0;}
  
  public static class ArmConstants {
    public static final int ARM_MOTOR_ID = 3;
    public static final double ARM_GEAR_RATIO = 1.0; // Adjust as needed
    public static final double ARM_MAX_ANGLE = 90.0; // Maximum angle in degrees
    public static final double ARM_MIN_ANGLE = 0.0; // Minimum angle in degrees
    public static final double ARM_SPEED = 1.0; // Speed of the arm movement
  }
}
