package frc.robot.subsystems.Arm;

import frc.robot.Constants;
import frc.robot.Constants.ARM;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;


public class Arm extends SubsystemBase{

    
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static boolean hasInstance = false;   

    private Rotation2d armSetPoint = new Rotation2d();

    //private final LoggedTunableNumber armKP = new LoggedTunableNumber("Shooter/ARM/kP", Constants.ARM.KP);
    //private final LoggedTunableNumber armKI = new LoggedTunableNumber("Shooter/ARM/kI", Constants.ARM.KI);
    //private final LoggedTunableNumber armKD = new LoggedTunableNumber("Shooter/ARM/kD", Constants.ARM.KD);
    //private final LoggedTunableNumber armKG = new LoggedTunableNumber("Shooter/ARM/kG", Constants.ARM.KG);

    //private boolean flipper = false;

    private Arm(ArmIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of arm already exists");
        hasInstance = true;
        this.armIO = io;
    }


    @Override
    public void periodic() {
        // Update & process inputs

        //from Shooter.java 
        armIO.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        //MechanismVisualiser.setArmRotation(getArmPosition());

        //if (RobotState.isDisabled()){
        //    stopArm();
        //}

        // Update tunable numbers
        //if (Constants.isTuningMode() && (armKP.hasChanged(hashCode()) || armKI.hasChanged(hashCode()) || armKD.hasChanged(hashCode()) || armKG.hasChanged(hashCode()))) {
        //    armIO.configPID(armKP.get(), armKI.get(), armKD.get(), armKG.get());
        //}

        // Add values to filters

   
        Logger.recordOutput("Shooter/Pivot/Error", armSetPoint.getDegrees() - inputs.motorAngle.getDegrees());
    }


    /**
     * Sets the position of the arm
     */
    public void setArmPosition(Rotation2d position) {
        armSetPoint = Rotation2d.fromRotations(
                MathUtil.clamp(
                        position.getRotations(),
                        ARM.MIN_ANGLE.getRotations(),
                        ARM.MAX_ANGLE.getRotations()
                )
        );
        //armPositionFilter.clear();
        inputs.setAngle = armSetPoint;
        inputs.setRotations = armSetPoint.getRotations();
        armIO.setPosition(armSetPoint);
    }
    /**
     * Sets the voltage to the arm
     */
    public void setArmVolts(double volts) {
        armIO.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    /**
     * Applies neutral output to the arm
     */
    public void stopArm() {
        setArmVolts(0);
    }

    public void setArmBrakeMode(boolean isBrakeMode) {
        armIO.setBrakeMode(isBrakeMode);
    }
    
    /**
     * Sets the position of the arm, using parallel to the floor as 0
     */
    public void holdArmPosition(){
        armIO.setPosition(armSetPoint);
    }

    /**
     * Gets the position of the arm
     */
    public Rotation2d getArmPosition() {
        return inputs.motorAngle;
    }

    /**
     * Gets whether the arm is at its setpoint
     */
    public boolean isArmAtSetpoint() {
        return MathUtil.isNear(
            armSetPoint.getRotations(), inputs.motorAngle.getRotations(), ARM.ERROR_TOLERANCE.getRotations());
            
    }

    public Command setArmPositionCommand(Supplier<Rotation2d> setpoint) {
        return new FunctionalCommand(
                () -> {},
                () -> setArmPosition(setpoint.get()),
                (ignored) -> {},
                this::isArmAtSetpoint,
                this);
    }

//    public void counterClockwiseArmMove(Rotation2d angle){
//        armSetPoint.plus(angle);
//        setArmPosition(armSetPoint);
//    }

 //   public void clockwiseArmMove(Rotation2d angle){
 //       armSetPoint.plus(angle);
 //       setArmPosition(armSetPoint);
//    }

    public void addAngle(double changeRotations){
        double currentRotations = inputs.absoluteAngle.getRotations();
        Rotation2d tempAngle = Rotation2d.fromRotations(changeRotations+currentRotations);
        setArmPosition(tempAngle);
        inputs.newRotations = changeRotations + currentRotations;

    }

    @AutoLogOutput(key = "Arm/Setpoint")
    public Rotation2d getArmSetpoint(){
        return armSetPoint;
    }

    //@AutoLogOutput(key = "Arm/AbsolutePivotSetpoint")
    //public Rotation2d getAbsolutePivotSetpoint(){
        //return armSetPoint.plus(angleOffset);
    //}

    public void zeroPivot() {
        armIO.setEncoderPosition(0);
    }

    public void zeroArmToCancoder(){
        armIO.setEncoderPosition(inputs.absoluteRotations);

    }

    //Toggle the breakmode
    public void toggleArmBrakeMode(){
        boolean setMode = inputs.breakOn ? false : true;
        armIO.setBrakeMode(setMode);
        inputs.breakOn = setMode;

    }


    public static Arm createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real arm on simulated robot", false);
        }
        return new Arm(new ArmIOReal());
    }
    
    public static Arm createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated arm on real robot", false);
        }
        return new Arm(new ArmIOSim());
    }
    
    public static Arm createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy arm on real robot", false);
        }
        return new Arm(new ArmIO(){});
    }
    
}



