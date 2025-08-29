package frc.robot.subsystems.Rollers;
import frc.robot.Constants;
import frc.robot.Constants.ROLLERS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;



public class Rollers extends SubsystemBase{

    
    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private static boolean hasInstance = false;   
    
    private Rollers(RollersIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of rollers already exists");
        hasInstance = true;
        this.rollersIO = io;
    }

    /**
     * periodic function to save variable for Advantage scope
     */
    @Override
    public void periodic() {
        // Update & process inputs

        //from Shooter.java 
        rollersIO.updateInputs(inputs);
        Logger.processInputs("Rollers", inputs);
        //MechanismVisualiser.setRollersRotation(getRollersPosition());
    }



     /* Sets the speed to the rollers
     */
    public void setRollersSpeed(double speed) {
        rollersIO.setSpeed(MathUtil.clamp(speed, -1, 1));
    }

    /**
     * stops the rollers
     */
    public void stopRollers() {
        setRollersSpeed(0);
    }
    
    /**
     * sets breakmode
     */
    public void setRollersBrakeMode(boolean isBrakeMode) {
        rollersIO.setBrakeMode(isBrakeMode);
    }
    
    //Fable delete later
    public void openRollers(){
        setRollersSpeed(.3);

    }

    //Fable delete later
    public void closeRollers(){
        setRollersSpeed(ROLLERS.ClOSE_SPEED);
    }


    /**
     * rotates intake to get a coral off of the floor
     */
    public void getCoral(){
        //spin both motors
        rollersIO.getCoral();
    }

    /**
     * rotates intake to shoot it out a coral
     */
    public void spitCoral(){
        //spin both motors
        rollersIO.spitCoral();
    }

     /**
     * rotates intake to get an algae off of the floor
     */
    public void getAlgae(){
        //spin both motors
        rollersIO.getAlgae();
    }

     /**
     * rotates intake to shoot it out an algae
     */
    public void spitAlgae(){
        //spin both motors
        rollersIO.spitAlgae();
    }

     /**
     * rotates intake to shoot it out an algae
     */
    public void knockAlgae(){
        //spin both motors
        rollersIO.knockAlgae();
    }

    public boolean isCoralLoaded(){
        //rollersIO.isCoralLoaded(a);
         return inputs.isCoralLoaded;
    }

    public boolean isAlgaeLoaded(){
        //rollersIO.isCoralLoaded(a);
         return inputs.isAlgaeLoaded;
    }

    public void setAlgaeFinished(boolean flag){
        //rollersIO.isCoralLoaded(a);
         inputs.isAlgaeFinished=flag;
    }
    
    
    //?is the coral loaded

    //


    public static Rollers createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real rollers on simulated robot", false);
        }
        return new Rollers(new RollersIOReal());
    }
    
    public static Rollers createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated rollers on real robot", false);
        }
        return new Rollers(new RollersIOSim());
    }
    
    public static Rollers createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy rollers on real robot", false);
        }
        return new Rollers(new RollersIO(){});
    }
    
}



