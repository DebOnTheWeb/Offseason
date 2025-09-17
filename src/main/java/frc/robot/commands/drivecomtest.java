// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Drive.DriveConstants;
// import java.util.function.DoubleSupplier;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class drivecomtest extends Command {
//   /** Creates a new DriveCommand. */
//   public final DriveConstants drivetrain;
//   private final DoubleSupplier forwardSupplier;
//   private final DoubleSupplier rotationSupplier;
//   public drivecomtest(
//   DriveConstants drivetrain,
//   DoubleSupplier forwardSupplier,
//   DoubleSupplier rotationSupplier) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.drivetrain = drivetrain;
//     this.forwardSupplier = forwardSupplier;
//     this.rotationSupplier = rotationSupplier;
//     addRequirements(drivetrain);//ensures no other commands run on drivetrain while this is active

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     System.out.println("TeleopDriveCommand started");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double forward = forwardSupplier.getAsDouble();
//     double rotation = rotationSupplier.getAsDouble();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     DriveConstants.Drive(0,0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
