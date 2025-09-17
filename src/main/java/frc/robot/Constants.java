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

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.led.patterns.LedPatternBreathing;
import edu.wpi.first.math.util.Units;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true; // TODO: set false for competition

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class LEDS {
    public static final int LED_STRIP_PORT_PWM = 9;
    public static final int LED_STRIP_LENGTH= 41;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

    public static final class ARM {
    public static final int RIGHT_MOTOR_ID = 20;
    public static final int LEFT_MOTOR_ID = 21;
    public static final int ABSOLUTE_ID = 25;

    // TODO: determine ratios
    //public static final double CANCODER_TO_PIVOT = 28.0 / 15.0;
    //public static final double MOTOR_TO_CANCODER = 56.0;
    //public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRadians(0.01215595 - 0.00548239159444);

    //public static final double CHAIN_FACTOR = 1.04;
    //public static final double TOTAL_GEAR_RATIO = MOTOR_TO_CANCODER * CANCODER_TO_PIVOT;
    //public static final double SIM_LENGTH_METERS = Units.inchesToMeters(12.910);
    // TODO: find the moi
    //public static final double SIM_MOI = 0.04064471269;

    public static final Rotation2d MAX_ANGLE = Rotation2d.fromRotations(0.298);//(.65);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromRotations(0.039);//(.35);

    public static final Rotation2d INTAKE_CORAL_ANGLE = Rotation2d.fromRotations(0.2985);//(.64);
    public static final Rotation2d PUT_CORAL_ANGLE = Rotation2d.fromRotations(0.193);//(0.532);
    public static final Rotation2d INTAKE_ALGAE_ANGLE = Rotation2d.fromRotations(0.2194);//(0.565);
    public static final Rotation2d REMOVE_ALGAE_ANGLE = Rotation2d.fromRotations(0.18);//(0.5121);//.447
    public static final Rotation2d ALGAE_LOCKED_ANGLE = Rotation2d.fromRotations(0.1914);//(.537); or 0.1902
    

    public static final double COPILOT_BUMPERS_ROTATIONS = 0.011;//.013888; 5 degrees

    //public static final Rotation2d REAL_ZEROED_ANGLE = Rotation2d.fromDegrees(8.5).minus(Rotation2d.fromRadians(0.3153));
    //public static final double PIVOT_HEIGHT = Units.inchesToMeters(10.5);

    public static final double KP = 200;
    public static final double KI = 3.0; //0.08;
    public static final double KD = 1.0;
    public static final double KS = 0.0;
    public static final double KG = 0.5;
    public static final double KV = 0.0;

    public static final double SIM_KP = 254;
    public static final double SIM_KI = 0.0;
    public static final double SIM_KD = 0.0;
    public static final double SIM_KS = 0.0;
    public static final double SIM_KG = 0.15;
    public static final double SIM_KV = 0.187;

    public static final double CRUISE_VELOCITY_RPS = 1.0;
    public static final double MAX_ACCEL_RPS2 = 2;
    public static final double JERK_RPS3 = 2000;

    public static final double SUPPLY_CURRENT_LIMIT=65;

    //public static final ShooterSetpoint HUB_SHOOT = new ShooterSetpoint(Rotation2d.fromRadians(1.06184));
    //public static final ShooterSetpoint PODIUM_SHOOT = new ShooterSetpoint(0.07, 6000,5000);


    public static final Rotation2d ERROR_TOLERANCE = Rotation2d.fromDegrees(0.7);

    //LATCH POSITIONS
    public static final Rotation2d APPROACH_POSITION_ANGLE = Rotation2d.fromRotations(.57);
    public static final Rotation2d LATCH_POSITION_ANGLE = Rotation2d.fromRotations(0.1452);//(.475);
    public static final Rotation2d LIFT_POSITION_ANGLE = Rotation2d.fromRotations(0.203);//(0.19217);//(0.2202);//(.550);//latch position angle + 0.075
  }



  public static final class LATCH {
    public static final int MOTOR_ID = 40;
    public static final boolean LATCH_INVERTED = false;
    public static final int LATCH_MOTOR_CURRENT_LIMIT= 6;//10;

     public static final double driveMotorReduction = 25.0;
      //(50 * 17 * 45) / (14 * 27 * 15); // MAXSwerve with 14 pinion teeth and 22 spur teeth

    // Drive encoder configuration
     public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec
    

      public static final double OPEN_SPEED = 0.5;
      public static final double ClOSE_SPEED = -0.5;

      //create code for hook
  }
  public static final class ROLLERS {
    public static final int TOP_ID = 31;//right
    public static final int BOTTOM_ID = 30;//left
    public static final boolean TOP_ROLLERS_INVERTED = true;
    public static final boolean BOTTOM_ROLLERS_INVERTED = true;
    public static final int ROLLERS_MOTOR_CURRENT_LIMIT= 45;

     public static final double driveMotorReduction = 25.0;
      //(50 * 17 * 45) / (14 * 27 * 15); // MAXSwerve with 14 pinion teeth and 22 spur teeth

    // Drive encoder configuration
     public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec
    

      public static final double CORAL_INTAKE_SPEED = .5;
      public static final double CORAL_OUTTAKE_SPEED = 0.3;
      public static final double ALGAE_INTAKE_SPEED = .7;
      public static final double ALGEA_OUTTAKE_SPEED = 1;
      public static final double KNOCK_ALGAE_SPEED = .5;
      public static final double ClOSE_SPEED = -.3;
     
  }
    public static boolean isTuningMode() {
      return tuningMode && !DriverStation.isFMSAttached();
  }

}
