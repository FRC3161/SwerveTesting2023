package frc.robot.constants;

public class PID {
    public double kp;
    public double ki;
    public double kd;
    public double kff;

    public PID(double kp, double ki, double kd, double kff) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kff = kff;
    }
}
