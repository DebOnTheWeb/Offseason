package frc.robot.subsystems.Rollers;
// store interesting variables. interesting ways  can be used
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    class RollersIOInputs {
        public double topMotorPosition = 0.0;
        public double topMotorVelocityRPM = 0.0;
        public double topMotorCurrentAmps = 0.0;
        public double bottomMotorPosition = 0.0;
        public double bottomMotorVelocityRPM = 0.0;
        public double bottomMotorCurrentAmps = 0.0;
        public boolean isCoralLoaded = false;
        public boolean isAlgaeLoaded = false;
        public boolean isAlgaeFinished = false;




    }


    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(RollersIOInputs inputs) {}



    /**
     * Runs open loop at the specified voltage
     */
    default void setSpeed(double speed) {}

    /**
     * Sets the neutral output mode
     */
    default void setBrakeMode(boolean isBrakeMode) {}

    /**
     * rotates intake to get a coral off of the floor
     */
    default void getCoral(){
        //spin both motors
        //setRollersSpeed= 
    }

     /**
     * rotates intake to to shoot it out a coral
     */
    default void spitCoral(){
        //spin both motors
        
    }

      /**
     * rotates intake to get a coral off of the floor
     */
    default void getAlgae(){

        //spin both motors
        
    }

     /**
     * rotates intake to shoot it out an algae
     */
    default void spitAlgae(){
        //spin both motors
        
    }

     /**
     * rotates intake to shoot it out an algae
     */
    default void knockAlgae(){
        //spin both motors
        
    }

}