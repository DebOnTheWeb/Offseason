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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.allStop;
import frc.robot.commands.Arm.approachPosition;
import frc.robot.commands.Arm.armBreakModeToggle;
import frc.robot.commands.Arm.armIntakeAlgae;
import frc.robot.commands.Arm.armIntakeCoral;
import frc.robot.commands.Arm.armPlaceCoral;
import frc.robot.commands.Arm.armRemoveAlgae;
import frc.robot.commands.Arm.latchPosition;
import frc.robot.commands.Arm.liftPosition;
import frc.robot.commands.Arm.lowerArm;
import frc.robot.commands.Arm.raiseArm;
import frc.robot.commands.Latch.closeLatch;
import frc.robot.commands.Latch.openLatch;
import frc.robot.commands.Rollers.openRollers;
import frc.robot.commands.Rollers.spitAlgae;
import frc.robot.commands.Rollers.spitCoral;
import frc.robot.commands.Rollers.closeRollers;
import frc.robot.commands.Rollers.getAlgae;
import frc.robot.commands.Rollers.getCoral;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Latch.Latch;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.led.patterns.SimpleLedPattern;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.subsystems.Rollers.Rollers;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Latch latch;
  private final Rollers rollers;
  private final Arm arm;




  // Controllers
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController coPilotController = new CommandXboxController(1);


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //CameraServer.startAutomaticCapture();
    //CameraServer.startAutomaticCapture();
    UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setFPS(15);
    UsbCamera camera2 = CameraServer.startAutomaticCapture();
      camera2.setFPS(15);
        

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        latch = Latch.createReal();
        rollers = Rollers.createReal();
        arm = Arm.createReal();
        drive =
            new Drive(
                // new GyroIOPigeon2(), FWM changed from pigeon to NavX
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        latch = Latch.createSim();
        rollers = Rollers.createSim();
        arm = Arm.createSim();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        latch = Latch.createDummy();
        rollers = Rollers.createDummy();
        arm = Arm.createDummy();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    new EventTrigger("spitCoral").onTrue(new spitCoral(rollers));
    new EventTrigger("Arm Coral Angle").onTrue(new armPlaceCoral(arm));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

   //Pilot controller
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(//changes speed of drive*
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY()*.8,
            () -> -driveController.getLeftX()*.8,
            () -> -driveController.getRightX()*.8));//Slowed it down FWM
  

    // Lock to 0° when A button is held
    /*
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));

                */
    // Switch to X pattern when X button is pressed
    //driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    /* 
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
                    }*/


    //driveController.x().onTrue(new closeRollers(rollers));// remove driveController
    //driveController.b().onTrue(new openRollers(rollers));// remove driveController
    //Latch commands

    // Pilot Controls
    driveController.start().onTrue(new allStop(rollers,latch));//commands all stop 
    //roller commands
    driveController.rightBumper().onTrue(new getCoral(rollers));
    driveController.rightTrigger().onTrue(new spitCoral(rollers));
    driveController.leftBumper().onTrue(new getAlgae(rollers, arm));
    driveController.leftTrigger().onTrue(new spitAlgae(rollers, arm));
    //arm commands
    driveController.a().onTrue(new armIntakeCoral(arm));
    driveController.b().onTrue(new armPlaceCoral(arm));
    driveController.x().onTrue(new armIntakeAlgae(arm));
    driveController.y().onTrue(new armRemoveAlgae(arm));

    // Co-Pilot Controls
    coPilotController.start().onTrue(new allStop(rollers,latch));//commands all stop
    //Manual Arm commands
    coPilotController.rightBumper().onTrue(new raiseArm(arm));
    coPilotController.leftBumper().onTrue(new lowerArm(arm));
    //coPilotController.back().onTrue(new armBreakModeToggle(arm));
    //Latch commands
    coPilotController.a().onTrue(new approachPosition(arm));
    coPilotController.b().onTrue(new latchPosition(arm));
    coPilotController.y().onTrue(new liftPosition(arm));
    coPilotController.leftTrigger().onTrue(new openLatch(latch));
    coPilotController.rightTrigger().onTrue(new closeLatch(latch));
    //Co-Pilot Joystick comamands
     // Switch to X pattern when X button is pressed
    coPilotController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
