package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.PID;

public class RelativeMotor extends Motor {
    public RelativeEncoder encoder;

    public RelativeMotor(CANSparkMax controller, PID pid) {
        super(controller, pid);
        this.encoder = this.controller.getEncoder();
    }
}
