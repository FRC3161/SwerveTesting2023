package frc.lib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SVAConstants {
    public double s, v, a;
    private String subscript;

    public SVAConstants(double s, double v, double a) {
        this.s = s;
        this.v = v;
        this.a = a;
    }

    public SimpleMotorFeedforward getController() {
        return new SimpleMotorFeedforward(this.s, this.v, this.a);
    }

    public void sendDashboard(String subscript) {
        this.subscript = subscript;
        SmartDashboard.putNumber(subscript + " S", this.s);
        SmartDashboard.putNumber(subscript + " V", this.v);
        SmartDashboard.putNumber(subscript + " A", this.a);
    }

    public SimpleMotorFeedforward retrieveDashboard() {
        this.s = SmartDashboard.getNumber(this.subscript + " S", 0.0);
        this.v = SmartDashboard.getNumber(this.subscript + " V", 0.0);
        this.a = SmartDashboard.getNumber(this.subscript + " A", 0.0);
        return this.getController();
    }
}
