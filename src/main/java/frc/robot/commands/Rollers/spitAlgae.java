// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Rollers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ARM;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Rollers.Rollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class spitAlgae extends Command {
  /** Creates a new spitAlgae. */
  
  private final Arm arm;
  
  private final Rollers intake;
  double timeStarted;
  public spitAlgae(Rollers rollers, Arm arm) {
    this.arm = arm;
    intake = rollers;
    addRequirements(rollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = Timer.getFPGATimestamp();
    intake.spitAlgae();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmPosition(ARM.ALGAE_LOCKED_ANGLE);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( Timer.getFPGATimestamp() > timeStarted+0.5){
      return true;
    }else{
      return false;
    }

  }
}
