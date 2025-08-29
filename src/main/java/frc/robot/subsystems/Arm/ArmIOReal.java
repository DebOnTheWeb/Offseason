package frc.robot.subsystems.Arm;
//program used for comps
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ARM;
import frc.robot.Constants.ARM;

import static frc.robot.Constants.ARM;

public class ArmIOReal implements ArmIO {
    private final TalonFX rightMotor = new TalonFX(ARM.RIGHT_MOTOR_ID);
    private final TalonFX leftMotor = new TalonFX(ARM.LEFT_MOTOR_ID);
    //private final CANcoder cancoder = new CANcoder(CANCODER_ID);
    private final StatusSignal<Angle> position = rightMotor.getPosition();
    //private final StatusSignal<Double> absolutePosition = cancoder.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> velocity = rightMotor.getVelocity();
    private final StatusSignal<Voltage> appliedVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> current = rightMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> temp = rightMotor.getDeviceTemp();
    private final StatusSignal<ForwardLimitValue> forwardLimit = rightMotor.getForwardLimit();
    private final StatusSignal<ReverseLimitValue> reverseLimit = rightMotor.getReverseLimit();

    private final MotionMagicVoltage positionCtrlReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0);
    //private Rotation2d setPosition = new Rotation2d();
    //private Rotation2d addPosition = new Rotation2d();
    private final Follower followerReq = new Follower(rightMotor.getDeviceID(), true);

    private final SparkMax motor;
    private final AbsoluteEncoder absEncoder;
    //public double absolutePosition2

    public ArmIOReal() {
        //configure absolute encoder
        
        motor = new SparkMax(ARM.ABSOLUTE_ID, MotorType.kBrushless);
        absEncoder = motor.getAbsoluteEncoder();

        var driveConfig = new SparkMaxConfig();
        driveConfig
            .signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20)
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20);
           

        motor.configure(driveConfig ,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        //Configure the Talon Motors
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightMotorConfig.Feedback.SensorToMechanismRatio =249.27562;
        rightMotorConfig.Feedback.RotorToSensorRatio = 1;


        //rightMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        //rightMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ARM.MAX_ANGLE.getRotations();
        //rightMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        //rightMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ARM.MIN_ANGLE.getRotations();

        //PID values
        rightMotorConfig.Slot0.kP = ARM.KP;
        rightMotorConfig.Slot0.kI = ARM.KI;
        rightMotorConfig.Slot0.kD = ARM.KD;
        rightMotorConfig.Slot0.kS = ARM.KS;
        rightMotorConfig.Slot0.kG = ARM.KG;
        rightMotorConfig.Slot0.kV = ARM.KV;
        //rightMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        rightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ARM.CRUISE_VELOCITY_RPS;
        rightMotorConfig.MotionMagic.MotionMagicAcceleration = ARM.MAX_ACCEL_RPS2;
        rightMotorConfig.MotionMagic.MotionMagicJerk = ARM.JERK_RPS3;

        rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = ARM.SUPPLY_CURRENT_LIMIT;
        //rightMotorConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
        //rightMotorConfig.CurrentLimits.SupplyTimeThreshold = 15;

        
        rightMotor.getConfigurator().apply(rightMotorConfig);


        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            position,
            //absolutePosition,
            velocity,
            appliedVoltage,
            current,
            temp,
            forwardLimit,
            reverseLimit);

        rightMotor.optimizeBusUtilization();

        leftMotor.getConfigurator().apply(rightMotorConfig);
        leftMotor.optimizeBusUtilization();
        leftMotor.setControl(followerReq);
        rightMotor.setPosition(getAbsoluteRotations());
            //Rotation2d.fromRotations(position.getValueAsDouble() / ARM.CANCODER_TO_PIVOT * ARM.CHAIN_FACTOR).plus(ARM.ENCODER_OFFSET).getRotations()
       // );
        //setPosition = new Rotation2d(-Math.PI/2);
        rightMotor.setControl(positionCtrlReq.withPosition(getAbsoluteRotations()));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, current, temp, forwardLimit, reverseLimit);
        inputs.isAtForwardLimit = forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.isAtReverseLimit = reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();

        //get the postion of the absolute encoder
        inputs.absoluteRotations = getAbsoluteRotations();
        inputs.absoluteAngle = Rotation2d.fromRotations(inputs.absoluteRotations);
        //get the postion of the motor encoder
        inputs.motorRotations = inputs.motorAngle.getRotations();
        inputs.motorAngle = Rotation2d.fromRotations(position.getValueAsDouble());

    }

    @Override
    public void setPosition(Rotation2d position) {
        rightMotor.setControl(positionCtrlReq.withPosition(position.getRotations()));
    }

    @Override
    public void setVoltage(double volts) {
        //rightMotor.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        rightMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        leftMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kG) {
        Slot0Configs pidConfigs = new Slot0Configs();
        rightMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kG = kG;
        rightMotor.getConfigurator().apply(pidConfigs);
    }
    @Override
    public void setEncoderPosition(double rots) {
        rightMotor.setPosition(rots);
    }

    private double getAbsoluteRotations(){
        double tim;
        tim = absEncoder.getPosition();
        //if (tim <.5){
        //    tim+=1;
        //}
        return tim;
    
    }


}   
    


