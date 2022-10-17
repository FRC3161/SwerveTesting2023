package frc.robot.motors;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;

import frc.lib.utils.CANCoderUtil;
import frc.lib.utils.CANSparkMaxUtil;
import frc.lib.utils.CANCoderUtil.CANCoderUsage;
import frc.lib.utils.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.constants.PID;

public class PodMotor extends RelativeMotor {
    public WPI_CANCoder externalEncoder;

    public PodMotor(CANSparkMax controller, PID pid, WPI_CANCoder externalEncoder) {
        super(controller, pid);
        this.externalEncoder = externalEncoder;
        this.setup();
    }

    void setup() {
        /**
         * External encoder config
         */
        this.externalEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(this.externalEncoder, CANCoderUsage.kMinimal);
        CANCoderConfiguration externalEncoderConfig = new CANCoderConfiguration();
        externalEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        externalEncoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        externalEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        externalEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        this.externalEncoder.configAllSettings(externalEncoderConfig);

        /**
         * Controller (motor) config
         */
        this.controller.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(this.controller, Usage.kPositionOnly);
        this.controller.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        this.controller.setInverted(Constants.Swerve.angleInvert);
        this.controller.setIdleMode(Constants.Swerve.angleNeutralMode);
        this.encoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        this.pid.updatePIDController(this.PIDcontroller);
        this.controller.enableVoltageCompensation(Constants.Swerve.voltageComp);
        this.controller.burnFlash();
    }
}
