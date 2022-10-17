package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.constants.PID;

public class RelativeMotor extends Motor {
    public RelativeEncoder encoder;
    public SparkMaxPIDController PIDcontroller;

    public RelativeMotor(CANSparkMax controller, PID pid) {
        super(controller, pid);
        this.encoder = this.controller.getEncoder();
        this.PIDcontroller = this.controller.getPIDController();
    }
}
