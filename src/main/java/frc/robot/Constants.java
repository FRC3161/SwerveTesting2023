package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.*;

public class Constants {
    public static SwerveModuleConstants[] swerveModules = {
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 4, 0, 0.0),
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 5, 1, 0.0),
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 6, 2, 0.0),
            new SwerveModuleConstants(new PID(0.0, 0.0, 0.0, 0.0), new PID(0.0, 0.0, 0.0, 0.0), 0, 7, 3, 0.0)
    };

    public static class Swerve {
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1

        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;
        public static final boolean canCoderInvert = false;
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;
        public static final boolean angleInvert = false;
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final double angleConversionFactor = 360.0 / 12.8;
        public static final double voltageComp = 12.0;
        public static final boolean driveInvert = false;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
        public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;

    }
}
