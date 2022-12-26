package frc.lib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class for managing the feedforward controllers and be able to make changes
 * that affect all loops in the robot such as live edit
 */
public class SVAConstants {
    public double s, v, a;
    private String subscript;

    public SVAConstants(double s, double v, double a) {
        this.s = s;
        this.v = v;
        this.a = a;
    }

    /* A function to get the controller based on the class values */
    public SimpleMotorFeedforward getController() {
        return new SimpleMotorFeedforward(this.s, this.v, this.a);
    }

    /* Send the values to the dashbaord */
    public void sendDashboard(String subscript) {
        this.subscript = subscript;
        SmartDashboard.putNumber(subscript + " S", this.s);
        SmartDashboard.putNumber(subscript + " V", this.v);
        SmartDashboard.putNumber(subscript + " A", this.a);
    }

    /* Get the values from the dashboard and update them */
    public SimpleMotorFeedforward retrieveDashboard() {
        this.s = SmartDashboard.getNumber(this.subscript + " S", 0.0);
        this.v = SmartDashboard.getNumber(this.subscript + " V", 0.0);
        this.a = SmartDashboard.getNumber(this.subscript + " A", 0.0);
        return this.getController();
    }
}
