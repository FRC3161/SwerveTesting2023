package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.PIDConstants;
import frc.lib.util.SVAConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final class Dashboard {
    public static final ShuffleboardTab generalTab = Shuffleboard.getTab("General");
    public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  }

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 21;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23);
    public static final double wheelBase = Units.inchesToMeters(23);
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = ((150 / 7) / 1.0); // 150/7:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // Front Left
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front Right
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Back Left
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // Back Right
    );

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 40;

    /* Angle Motor PID Values */
    public static double angleKP = 0.01;
    public static double angleKI = 0.0;
    public static double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;
    public static final double angleConversionFactor = 360.0 / (150 / 7);

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    public static final String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 23;
      public static final String cancoderCANBUS = "canivore3161";
      public static final double angleOffset = 146.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.0, 0.0, 0.000);
      public static final SVAConstants driveSVA = new SVAConstants(0.667, 2.44, 0.27);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 24;
      public static final String cancoderCANBUS = "canivore3161";
      public static final double angleOffset = 324.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.1, 0.0, 0.005);
      public static final SVAConstants driveSVA = new SVAConstants(0.667, 2.44, 0.27);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 22;
      public static final String cancoderCANBUS = "canivore3161";
      public static final double angleOffset = 290.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.1, 0.0, 0.005);
      public static final SVAConstants driveSVA = new SVAConstants(0.667, 2.44, 0.27);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 25;
      public static final String cancoderCANBUS = "canivore3161";
      public static final double angleOffset = 90.0;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final PIDConstants drivePID = new PIDConstants(0.1, 0.0, 0.005);
      public static final SVAConstants driveSVA = new SVAConstants(0.667, 2.44, 0.27);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
