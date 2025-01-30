// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4.5; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // DRIVE CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 1;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = 0.10471975803375244; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = 0.10471975803375244; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.08;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 25; // amps
  }

  public static final class ControllerConstants {
      public static final int kDriverControllerPort = 0;
      public static final double kDriveDeadband = 0.10;

      public final static int k_start = Button.kStart.value; // Start Button
      public static final int k_A = Button.kA.value; // A
      public static final int k_B = Button.kB.value; // B
      public static final int k_X = Button.kX.value; // X
      public static final int k_Y = Button.kY.value; // Y
  }

  public static final class AutoConstants {
    public static final double AutoSpeed = 0.5;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {

    // Name
    public static final String k_limelightName = "limelight-three";

    // PID for Tag Relative Control in General
    public static final double kP_aim = 0.04;
    public static final double kI_aim = 0.000;
    public static final double kD_aim = 0.000;

    public static final double kP_range = 0.13;
    public static final double kI_range = 0.0;
    public static final double kD_range = 0.0;

    public static final double kP_strafe = 0.13;
    public static final double kI_strafe = 0.0;
    public static final double kD_strafe = 0.0;

    // Aim/Range
    public static final double k_aimThreshold = 0.5;
    public static final double k_rangeThresholdMax = -3.8;
    public static final double k_rangeThresholdMin = -4.2;
    public static final double k_rangeTarget = -4;

    // AimNRange Reef Right
    public static final double k_aimReefRightThresholdMax = 0.5;
    public static final double k_aimReefRightThresholdMin = -0.5;
    public static final double k_aimReefRightTarget = 0;

    public static final double k_rangeReefRightThresholdMax = -0.63;
    public static final double k_rangeReefRightThresholdMin = -0.68;
    public static final double k_rangeReefRightTarget = -0.65;

    public static final double k_strafeReefRightThresholdMax = 0.20;
    public static final double k_strafeReefRightThresholdMin = 0.15;
    public static final double k_strafeReefRightTarget = 0.18;

    // AimNRange Reef Left
    public static final double k_aimReefLeftThresholdMax = 0.5;
    public static final double k_aimReefLeftThresholdMin = -0.5;
    public static final double k_aimReefLeftTarget = 0;

    public static final double k_rangeReefLeftThresholdMax = -0.63;
    public static final double k_rangeReefLeftThresholdMin = -0.68;
    public static final double k_rangeReefLeftTarget = -0.65;

    public static final double k_strafeReefLeftThresholdMax = -0.15;
    public static final double k_strafeReefLeftThresholdMin = -0.20;
    public static final double k_strafeReefLeftTarget = -0.18;
  }
}