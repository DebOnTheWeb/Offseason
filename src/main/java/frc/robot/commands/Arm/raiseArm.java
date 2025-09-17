package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ARM;
import frc.robot.subsystems.Arm.Arm;

public class raiseArm extends Command{
    private final Arm arm;

    public raiseArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {    
        arm.addAngle(ARM.COPILOT_BUMPERS_ROTATIONS*-1);
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

