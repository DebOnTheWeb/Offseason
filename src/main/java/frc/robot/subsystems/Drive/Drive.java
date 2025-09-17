// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.configs.Pigeon2Configurator;
// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.ctre.phoenix6.swerve.SwerveDrivetrain;
// import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.kinematics.*;

// ;

// public class Drive extends SubsystemBase {
//   /** Creates a new Drive. */
//   private final SwerveDriveKinematics kinematics =
//     new SwerveDriveKinematics(DriveConstants.FRONT_LEFT_LOCATION,
//                               DriveConstants.FRONT_RIGHT_LOCATION,
//                               DriveConstants.BACK_LEFT_LOCATION,
//                               DriveConstants.BACK_RIGHT_LOCATION);


//   public double getYaw() {
//     return DriveConstants.kPigeonId;
//   }
//   public static final int gyro = DriveConstants.kPigeonId;

//   private final SwerveDriveOdometry kinematicsOdometry = 
//   new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d(0, 0, new Rotation2d()));
  
  
//     private final SwerveDriveOdometry odometry =
//     new SwerveDriveOdometry(kinematics, SwerveDrivetrainConstants.Pigeon2Configs, new Pose2d(0, 0, new Rotation2d()));
//   public Drive() {}

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     // ChassisSpeeds speeds = new ChassisSpeeds();//**MUST CHANGE TO FUCKING BIG
//     // Rotation2d rotation = Rotation2d.fromRotations(kCoupleRatio);
//     // SwerveModuleState[] moduleStates = kinematics.toSwerveModuleState(speeds);
    
//   }
// }
