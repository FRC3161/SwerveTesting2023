package frc.robot.motors;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;

import frc.robot.constants.PID;

public class AbsoluteMotor extends Motor {
    WPI_CANCoder encoder;

    public AbsoluteMotor(CANSparkMax controller, PID pid, WPI_CANCoder encoder) {
        super(controller, pid);
        this.encoder = encoder;
    }
}
