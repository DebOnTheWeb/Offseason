// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Drive.Drive;
// import java.util.function.DoubleSupplier;
// import java.util.function.BooleanSupplier;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class DriveCommandTwo extends Command {
//   /** Creates a new DriveCommand. */
//   private final Drive drivetrain;
//   private final DoubleSupplier xSupplier;
//   private final DoubleSupplier ySupplier;
//   private final DoubleSupplier rotationSupplier;
//   private final BooleanSupplier fieldRelativeSupplier;
//   public DriveCommandTwo(
//     Drive subsystem,
//     DoubleSupplier x,
//     DoubleSupplier y,
//     DoubleSupplier rot,
//     BooleanSupplier fieldRelative
//   ) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     drivetrain = subsystem;
//     xSupplier = x;
//     ySupplier = y;
//     rotationSupplier = rot;
//     fieldRelativeSupplier = fieldRelative;
//     addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     drivetrain.SwerveDrive(
//       xSupplier.getAsDouble(),
//       ySupplier.getAsDouble(),
//       rotationSupplier.getAsDouble(),
//       fieldRelativeSupplier.getAsBoolean()
//     );
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.SwerveDrive(0, 0, 0, interrupted);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
