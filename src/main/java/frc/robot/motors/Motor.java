package frc.robot.motors;

import com.revrobotics.CANSparkMax;

import frc.robot.constants.PID;

public class Motor {
    public CANSparkMax controller;
    public PID pid;

    public Motor(CANSparkMax controller, PID pid) {
        this.controller = controller;
        this.pid = pid;
    }
}
