// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Rollers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ARM;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Rollers.Rollers;
import frc.robot.subsystems.led.Leds.PatternLevel;
import frc.robot.subsystems.led.patterns.LedPatternBreathing;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class getAlgae extends Command {
  private final Rollers intake;
  private final Arm arm;
  double timeStarted;
  //double algaeDuration;
  //private boolean isAlgaeLocked = false;
  //private boolean stopping = false;
  
  /** Creates a new getAlgae. */
  public getAlgae(Rollers rollers, Arm arm) {
    this.arm = arm;
    intake = rollers;
    addRequirements(rollers);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = Timer.getFPGATimestamp();
    //intake.getAlgae();
    //intake.setAlgaeFinished(false);
    //stopping = false;
    //isAlgaeLocked = false;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.getAlgae();
  //if ( Timer.getFPGATimestamp() > timeStarted+1){
  //  isAlgaeLocked = intake.isAlgaeLoaded(); 
  //  if (isAlgaeLocked){
  //    arm.setArmPosition(ARM.ALGAE_LOCKED_ANGLE);
  //    algaeDuration = Timer.getFPGATimestamp();
  //    stopping = true;
  //  }

   // }
//     setPattern(Zone zone, LedPattern pattern) {
//     setPattern(zone, pattern, PatternLevel.INTAKE_STATE);
// }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
    //intake.setAlgaeFinished(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ( Timer.getFPGATimestamp() > timeStarted+1){
      return intake.isAlgaeLoaded();
    }else{
      return false;
    }

    //if (stopping &&  Timer.getFPGATimestamp() > algaeDuration + 0.4){
    //  intake.setAlgaeFinished(true);
    //  return true;
    //}
//
    //else{
    //  return false;
    //}
  
  }
}
