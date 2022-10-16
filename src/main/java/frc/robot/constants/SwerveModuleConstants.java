package frc.robot.constants;

public class SwerveModuleConstants {
    public PID wheelPID;
    public PID podPID;

    public int podEncoder;
    public int podController;
    public int wheelController;

    public SwerveModuleConstants(PID wheelPID, PID podPID, int podEncoder, int podController, int wheelController) {
        this.wheelPID = wheelPID;
        this.podPID = podPID;
        this.podEncoder = podEncoder;
        this.podController = podController;
        this.wheelController = wheelController;
    }
}
