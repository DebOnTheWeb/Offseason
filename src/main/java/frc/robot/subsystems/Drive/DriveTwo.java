// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;
// import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.swerve.SwerveDrivetrain;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Drive.DriveConstants;
// import frc.robot.subsystems.Drive.DriveConstants.TunerSwerveDrivetrain;


// public class DriveTwo extends SubsystemBase {
//   /** Creates a new Drive. */
//   private final TunerSwerveDrivetrain drivetrainConstants;
//   public DriveTwo(TunerSwerveDrivetrain driveConstants,SwerveModuleConstants[] moduleConstants) {
//     SwerveModule[] modules = new SwerveModule[moduleConstants.length];
//     for (int i = 0; i < moduleConstants.length; i++) {
//       modules[i] = new SwerveModule(moduleConstants[i]);
//       //new SwerveModule(moduleConstants[i]);
//     }
//    // drivetrainConstants = new SwerveDrivetrainConstants(driveConstants, modules);
//     drivetrainConstants = new TunerSwerveDrivetrain(driveConstants, modules);
//   }
    

//   public void SwerveDrive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
//     // This method will be called to drive the swerve drivetrain
//     // xSpeed and ySpeed are in meters per second
//     // rotation is in radians per second
//     // fieldRelative indicates whether the drive is relative to the field or robot orientation
//     drivetrainConstants.drive(xSpeed, ySpeed, rotation, fieldRelative);
//   }

//   @Override//SHOULD EXIST
//   public void periodic() {
//     This method will be called once per scheduler run
//     drivetrainConstants.odometryUpdateFrequency;
//   }
// }
