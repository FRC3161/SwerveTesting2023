package frc.robot.motors;

import com.revrobotics.CANSparkMax;

import frc.lib.utils.CANSparkMaxUtil;
import frc.lib.utils.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.constants.PID;

public class WheelMotor extends RelativeMotor {
    public WheelMotor(CANSparkMax controller, PID pid) {
        super(controller, pid);
        this.setup();
    }

    void setup() {
        /**
         * Wheel controller (motor) configurations
         */
        this.controller.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(this.controller, Usage.kVelocityOnly);
        this.controller.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        this.controller.setInverted(Constants.Swerve.driveInvert);
        this.controller.setIdleMode(Constants.Swerve.driveNeutralMode);
        this.encoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        this.pid.updatePIDController(this.PIDcontroller);
        this.controller.enableVoltageCompensation(Constants.Swerve.voltageComp);
        this.controller.burnFlash();
        this.encoder.setPosition(0.0);
    }
}
