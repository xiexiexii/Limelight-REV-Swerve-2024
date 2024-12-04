// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
      public static final int kFrontLeftDrivingCanId = 3;
      public static final int kRearLeftDrivingCanId = 2;
      public static final int kFrontRightDrivingCanId = 7;
      public static final int kRearRightDrivingCanId = 6;

      public static final int kFrontLeftTurningCanId = 4;
      public static final int kRearLeftTurningCanId = 1;
      public static final int kFrontRightTurningCanId = 8;
      public static final int kRearRightTurningCanId = 5;

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

  public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
      public static final double AutoSpeed = 0.5;
  }

  public static final class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LEDConstants {
    public static final int blinkinPort = 0;  
  }

  public static final class Controls {
    //driver controls
        public static final int setXValue = 1; //A

        /*
        //claw
        public static final int clawIntakeAxis = 0; //right trigger
        public static final int clawOuttakeAxis = 0; //left trigger

        //arm positions
        public static final int armSetToClimbButton = 0; //Y
        public static final int armSetToGroundButton = 0; //A
        public static final int armSetToPassButton = 0; //B

        //climbing
        public static final int climbForwardButton = 0; //R bumper
        public static final int climbBackwardButton = 0; //L bumper
        */
  }
}