package frc.robot.subsystems.Arm;
// store interesting variables. interesting ways  can be used
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public Rotation2d motorAngle = new Rotation2d();  //the position of the arm according to the motor in radians
        public Rotation2d setAngle = new Rotation2d(); //the position we want the arm to be in radians
        public Rotation2d absoluteAngle = new Rotation2d(); //the position of the arm according to the absolute encoder in radians
        //public Rotation2d addPosition = new Rotation2d(); //the angle we added to the set angle
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean isAtForwardLimit = false;
        public boolean isAtReverseLimit = false;
        public double motorRotations = 0.0;  //the position of the arm according to the motor in rotations
        public double setRotations = 0.0; //the position we want the arm to be in radians
        public double absoluteRotations = 0.0; //the position of the arm according to the absolute encoder in rotations
        // breakmode
        public boolean breakOn = true;

        //temp
        public double newRotations=0.0;
    }


    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(ArmIOInputs inputs) {}

    /**
     * Runs closed loop to the specified position
     */
    default void setPosition(Rotation2d position) {}



    /**
     * Runs open loop at the specified voltage
     */
    default void setVoltage(double volts) {}

    /**
     * Sets the neutral output mode
     */
    default void setBrakeMode(boolean isBrakeMode) {}

    /**
     * Configures the PID controller
     */
    default void configPID(double kP, double kI, double kD, double kG) {}

    default void setEncoderPosition(double rots) {}


}
    
