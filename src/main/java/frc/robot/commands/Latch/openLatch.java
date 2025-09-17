package frc.robot.commands.Latch;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Latch.Latch;

public class openLatch extends Command{
    private final Latch latch;
    double timeStarted;

    public openLatch(Latch latch) {
        this.latch = latch;
        
        addRequirements(latch);
    }

    @Override
    public void initialize() {
    timeStarted = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
       latch.openLatch();
    }

    @Override
    public boolean isFinished() {
        if ( Timer.getFPGATimestamp() > timeStarted+1){
            return true;
          }else{
            return false;
          }
    }

    @Override
    public void end(boolean interrupted) {

        latch.stopLatch();
    }
}
