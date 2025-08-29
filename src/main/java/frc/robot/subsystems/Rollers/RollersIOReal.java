package frc.robot.subsystems.Rollers;
//program used for comps
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ROLLERS;

import static frc.robot.Constants.ROLLERS;

public class RollersIOReal implements RollersIO {
    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private double bottomCurrent;
    private double bottomVelocity;
    private double topCurrent;
    private double topVelocity;

    private boolean topLoaded = false;
    private boolean bottomLoaded = false;


    public RollersIOReal() {
        //configure topMoter
        topMotor = new SparkMax(ROLLERS.TOP_ID, MotorType.kBrushless);
        topEncoder = topMotor.getEncoder();

        var topDriveConfig = new SparkMaxConfig();
        topDriveConfig
            .idleMode(IdleMode.kBrake) // FWM IdleMode.kBrake
            .inverted(ROLLERS.TOP_ROLLERS_INVERTED)
            .smartCurrentLimit(ROLLERS.ROLLERS_MOTOR_CURRENT_LIMIT);
        topDriveConfig
            .encoder
            .positionConversionFactor(ROLLERS.driveEncoderPositionFactor)
            .velocityConversionFactor(ROLLERS.driveEncoderVelocityFactor);
        topDriveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        topMotor.configure(topDriveConfig ,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        topEncoder.setPosition(0);

        //configure bottomMoter
        bottomMotor = new SparkMax(ROLLERS.BOTTOM_ID, MotorType.kBrushless);
        bottomEncoder = bottomMotor.getEncoder();

        var bottomDriveConfig = new SparkMaxConfig();
        bottomDriveConfig
            .idleMode(IdleMode.kBrake) // FWM IdleMode.kBrake
            .inverted(ROLLERS.BOTTOM_ROLLERS_INVERTED)
            .smartCurrentLimit(ROLLERS.ROLLERS_MOTOR_CURRENT_LIMIT);
        bottomDriveConfig
            .encoder
            .positionConversionFactor(ROLLERS.driveEncoderPositionFactor)
            .velocityConversionFactor(ROLLERS.driveEncoderVelocityFactor);
        bottomDriveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        bottomMotor.configure(bottomDriveConfig ,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        bottomEncoder.setPosition(0);
        


    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
    // Update drive inputs
    inputs.topMotorPosition = topEncoder.getPosition();
    inputs.topMotorVelocityRPM = topEncoder.getVelocity();
    topVelocity = bottomEncoder.getVelocity();
    inputs.topMotorCurrentAmps = topMotor.getOutputCurrent();
    topCurrent = bottomMotor.getOutputCurrent();
    inputs.bottomMotorPosition = bottomEncoder.getPosition();
    bottomVelocity = bottomEncoder.getVelocity();
    inputs.bottomMotorVelocityRPM = bottomVelocity;
    bottomCurrent = bottomMotor.getOutputCurrent();
    inputs.bottomMotorCurrentAmps = bottomCurrent;
    
    //if velocity is between -2 and and the current is high then stop
    if (topVelocity<=5 && topVelocity >= -2 && topCurrent > 10){//top v is 2 because 
        topLoaded = true;
    }
    else{
        topLoaded = false;
    }
    inputs.isCoralLoaded = topLoaded;

    if (bottomVelocity<=0.5 && bottomVelocity >= -0.5 && bottomCurrent > 10){
        bottomLoaded = true;
    }
    else{
        bottomLoaded = false;
    }
    inputs.isAlgaeLoaded = bottomLoaded;

    }



    @Override
    public void setSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        var driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        topMotor.configure(driveConfig ,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        bottomMotor.configure(driveConfig ,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * rotates intake to get a coral off of the floor
     */
    
    @Override
    public void getCoral(){
        //spin both motors
        topMotor.set(ROLLERS.CORAL_INTAKE_SPEED);
        bottomMotor.set(ROLLERS.CORAL_INTAKE_SPEED);
    } 

    /**
     * rotates intake to shoot it out a coral
     */
    
     @Override
     public void spitCoral(){
        topMotor.set(ROLLERS.CORAL_OUTTAKE_SPEED*-1);
        bottomMotor.set(ROLLERS.CORAL_OUTTAKE_SPEED*-1);
     } 

     /**
     * rotates intake to get a algae off of the floor
     */
    
     @Override
     public void getAlgae(){
        bottomMotor.set(ROLLERS.ALGAE_INTAKE_SPEED*-1);
         
     } 
    
     /**
     * rotates intake to shoot it out a algae
     */

     @Override
     public void spitAlgae(){
        bottomMotor.set(ROLLERS.ALGEA_OUTTAKE_SPEED);
         
     }
     
      /**
     * rotates intake to shoot it out a algae
     */

     @Override
     public void knockAlgae(){
        topMotor.set(ROLLERS.KNOCK_ALGAE_SPEED*-1);
     }
    
}   
    


