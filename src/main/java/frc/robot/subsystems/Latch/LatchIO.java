package frc.robot.subsystems.Latch;
// store interesting variables. interesting ways  can be used
import org.littletonrobotics.junction.AutoLog;


public interface LatchIO {
    @AutoLog
    class LatchIOInputs {
        public double motorPosition = 0.0;
        public double motorVelocityRPM = 0.0;
        public double motorCurrentAmps = 0.0;



    }


    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(LatchIOInputs inputs) {}



    /**
     * Runs open loop at the specified voltage
     */
    default void setSpeed(double speed) {}

    /**
     * Sets the neutral output mode
     */
    default void setBrakeMode(boolean isBrakeMode) {}

}
    
