// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(15.1); // FWM 15.1
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(24.5); // FWM
  public static final double wheelBase = Units.inchesToMeters(24.5); // FWM
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions  FWM changed values
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(); //311.04 degrees
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(); //177.84 degrees
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(); //132.84 degrees
  public static final Rotation2d backRightZeroRotation = new Rotation2d(); //3.6 degrees

  //Added or subtracted
  public static final double frontLeftCanZero = .229220072;//.2245814;
  public static final double frontRightCanZero = -.048187;//-0.046387;
  public static final double backLeftCanZero = -.787100078;//-0.795645;
  public static final double backRightCanZero = -.414658281;//-0.41417;


  // Device CAN IDs
  public static final int pigeonCanId = 9;

  // FWM changed Can IDS for motors
  public static final int frontLeftDriveCanId = 8;
  public static final int backLeftDriveCanId = 7;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 6;

  public static final int frontLeftTurnCanId = 12;
  public static final int backLeftTurnCanId = 11;
  public static final int frontRightTurnCanId = 9;
  public static final int backRightTurnCanId = 10;

  public static final int frontLeftCancoderId = 4;
  public static final int backLeftCancoderId = 3;
  public static final int frontRightCancoderId = 1;
  public static final int backRightCancoderId = 2;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final boolean driveInverted = true; //fable from false
  public static final int driveMotorReverse = 1; // -1 true, 1 false
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0); // FWM
  // FWM public static final double driveMotorReduction =
  // FWM    (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final double driveMotorReduction = 6.71425;
      //(50 * 17 * 45) / (14 * 27 * 15); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec //60.0

  // Drive PID configuration
  public static final double driveKp = 0.0; //0.0
  public static final double driveKd = 0.0; //0.0
  public static final double driveKs = 0.0; //0.0
  public static final double driveKv = 0.3;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false; //fable from false

  public static final int turnMotorCurrentLimit = 30;
  // FWM public static final double turnMotorReduction = 9424.0 / 203.0;
  // public static final double turnMotorReduction = 150 / 7;
  //public static final double turnMotorReduction = 21.42857;
  public static final double turnMotorReduction = 21.37268736; //3rd time is the charm

  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final int turnMotorReverse = -1; // -1 true, 1 false
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Can Encoder configuration
  public static boolean EncoderInverted = true;

  // Turn PID configuration
  public static final double turnKp = .2;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = -Math.PI;//0; // Radians
  public static final double turnPIDMaxInput = Math.PI;//2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
