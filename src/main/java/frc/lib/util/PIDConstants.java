package frc.lib.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDConstants {
    public double p, i, d;
    private String subscript;

    public PIDConstants(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void applyPID(SparkMaxPIDController controller) {
        controller.setP(this.p);
        controller.setI(this.i);
        controller.setD(this.d);
    }

    public void sendDashboard(String subscript) {
        this.subscript = subscript;
        SmartDashboard.putNumber(subscript + " P", this.p);
        SmartDashboard.putNumber(subscript + " I", this.i);
        SmartDashboard.putNumber(subscript + " D", this.d);
    }

    public void retrieveDashboard(SparkMaxPIDController controller) {
        double p = SmartDashboard.getNumber(this.subscript + " P", 0.0);
        double i = SmartDashboard.getNumber(this.subscript + " I", 0.0);
        double d = SmartDashboard.getNumber(this.subscript + " D", 0.0);

        if (this.p != p) {
            controller.setP(p);
            this.p = p;
        }
        if (this.i != i) {
            controller.setI(i);
            this.i = i;
        }
        if (this.d != d) {
            controller.setD(d);
            this.d = d;
        }
    }
}
