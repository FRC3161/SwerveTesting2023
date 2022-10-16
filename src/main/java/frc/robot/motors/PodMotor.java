package frc.robot.motors;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;

import frc.robot.constants.PID;

public class PodMotor extends RelativeMotor {
    WPI_CANCoder externalEncoder;

    public PodMotor(CANSparkMax controller, PID pid, WPI_CANCoder externalEncoder) {
        super(controller, pid);
        this.externalEncoder = externalEncoder;
    }
}
