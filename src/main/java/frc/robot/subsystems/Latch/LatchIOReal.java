package frc.robot.subsystems.Latch;
//program used for comps
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.robot.Constants.LATCH;

public class LatchIOReal implements LatchIO {
    private final SparkMax motor;
    private final RelativeEncoder motorEncoder;

    public LatchIOReal() {
        motor = new SparkMax(LATCH.MOTOR_ID, MotorType.kBrushless);
        motorEncoder = motor.getEncoder();

        var driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake) // FWM IdleMode.kBrake
            .inverted(LATCH.LATCH_INVERTED)
            .smartCurrentLimit(LATCH.LATCH_MOTOR_CURRENT_LIMIT);
        driveConfig
            .encoder
            .positionConversionFactor(LATCH.driveEncoderPositionFactor)
            .velocityConversionFactor(LATCH.driveEncoderVelocityFactor);
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        motor.configure(driveConfig ,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorEncoder.setPosition(0);

    }

    @Override
    public void updateInputs(LatchIOInputs inputs) {
    // Update drive inputs
    inputs.motorPosition = motorEncoder.getPosition();
    inputs.motorVelocityRPM = motorEncoder.getVelocity();
    inputs.motorCurrentAmps = motor.getOutputCurrent();

    //add something to check hook
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        var driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        motor.configure(driveConfig ,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

}   
    


