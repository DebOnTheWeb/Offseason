package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ARM;
import frc.robot.subsystems.Arm.Arm;

public class lowerArm extends Command{
    private final Arm arm;

    public lowerArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {    
        arm.addAngle(ARM.COPILOT_BUMPERS_ROTATIONS);
    }

    @Override
    public void execute() {
       
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

        //arm.stopArm();
    }
}
