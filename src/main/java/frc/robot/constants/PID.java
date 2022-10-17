package frc.robot.constants;

import com.revrobotics.SparkMaxPIDController;

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

    public void updatePIDController(SparkMaxPIDController controller) {
        controller.setP(this.kp);
        controller.setI(this.ki);
        controller.setD(this.kd);
        controller.setFF(this.kff);
    }
}
