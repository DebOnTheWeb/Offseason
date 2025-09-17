package frc.robot.subsystems.Latch;
import frc.robot.Constants;
import frc.robot.Constants.LATCH;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;



public class Latch extends SubsystemBase{

    
    private final LatchIO latchIO;
    private final LatchIOInputsAutoLogged inputs = new LatchIOInputsAutoLogged();
    private static boolean hasInstance = false;   
    
    private Latch(LatchIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of latch already exists");
        hasInstance = true;
        this.latchIO = io;
    }


    @Override
    public void periodic() {
        // Update & process inputs

        //from Shooter.java 
        latchIO.updateInputs(inputs);
        Logger.processInputs("Latch", inputs);
        //MechanismVisualiser.setLatchRotation(getLatchPosition());

        //add something to check if its the end of the match
    }



     /* Sets the voltage to the latch
     */
    public void setLatchSpeed(double speed) {
        latchIO.setSpeed(MathUtil.clamp(speed, -1, 1));
    }

    /**
     * Applies neutral output to the latch
     */
    public void stopLatch() {
        setLatchSpeed(0);
    }

    public void setLatchBrakeMode(boolean isBrakeMode) {
        latchIO.setBrakeMode(isBrakeMode);
    }
    
    /**
     * Gets whether the latch is at its setpoint
     */
    public boolean isLatchLocked() {
        return false;
            
    }

    public void openLatch(){
        setLatchSpeed(LATCH.OPEN_SPEED);

    }

    public void closeLatch(){
        setLatchSpeed(LATCH.ClOSE_SPEED);
    }




    public static Latch createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real latch on simulated robot", false);
        }
        return new Latch(new LatchIOReal());
    }
    
    public static Latch createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated latch on real robot", false);
        }
        return new Latch(new LatchIOSim());
    }
    
    public static Latch createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy latch on real robot", false);
        }
        return new Latch(new LatchIO(){});
    }
    
}



