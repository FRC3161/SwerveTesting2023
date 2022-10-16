package frc.robot;

import frc.robot.constants.*;

public class Constants {
    public static SwerveModuleConstants[] swerveModules = {
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 4, 0, 0.0),
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 5, 1, 0.0),
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 6, 2, 0.0),
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 7, 3, 0.0)
    };
}
