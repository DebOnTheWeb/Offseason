package frc.robot.commands.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rollers.Rollers;

public class openRollers extends Command{
    private final Rollers rollers;

    public openRollers(Rollers rollers) {
        this.rollers = rollers;
        addRequirements(rollers);
    }

    @Override
    public void execute() {
       rollers.openRollers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        //rollers.stopRollers();
    }
}
