package frc.robot.subsystems;

import java.util.List;
import java.util.Map;

import javax.naming.spi.DirStateFactory.Result;

import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.Dashboard;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;
  private Rotation2d orientationWhenReleased;
  private Rotation2d orientationBeforeTarget;
  private double rotationControllerSpeed = 0.0;
  private final Field2d field = new Field2d();
  private final PIDController RobotRotationPID;
  private final PIDController TargetRotationPID;
  private final PIDController autoDrivePID;
  private final PhotonCamera camera = new PhotonCamera("gloworm");

  private NetworkTableEntry yawEntry;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID, "canivore3161");
    gyro.configFactoryDefault();
    zeroGyro();
    this.RobotRotationPID = new PIDController(0.1, 0, 0.00005);
    this.RobotRotationPID.enableContinuousInput(-180, 180);

    this.TargetRotationPID = new PIDController(6, 0, 0.05);
    this.TargetRotationPID.enableContinuousInput(2, 2);

    this.autoDrivePID = new PIDController(0.1, 0, 0);
    // this.RobotRotationPID.setTolerance(2.5);

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    this.yawEntry = Dashboard.generalTab.add("Gyroscope Yaw", this.getYaw().getDegrees())
        .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360)).getEntry();
  }

  public void rotationUpdate(double rotation) {
    if (this.rotationControllerSpeed != 0.0) {
      this.orientationWhenReleased = getYaw();
    }
    this.rotationControllerSpeed = rotation;
  }

  public double checkTargets(double missAlignment) {
    PhotonPipelineResult results = this.camera.getLatestResult();
    if (results.hasTargets()) {
      List<PhotonTrackedTarget> targets = results.getTargets();
      PhotonTrackedTarget target = targets.get(0);

      int targetID = target.getFiducialId();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      SmartDashboard.putNumber("target id", targetID);
      SmartDashboard.putNumber("target x", bestCameraToTarget.getX());
      SmartDashboard.putNumber("target Y", bestCameraToTarget.getY());
      this.orientationWhenReleased = this.getYaw();

      return this.TargetRotationPID.calculate(-bestCameraToTarget.getY(), 0);
    }

    return missAlignment;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
      boolean shoot) {
    this.rotationUpdate(rotation);
    double missalignment = MathUtil.clamp(
        this.RobotRotationPID.calculate(getYaw().getDegrees() % 360,
            this.orientationWhenReleased.getDegrees() % 360),
        -Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed);

    if (shoot) {
      // this.camera.setLED(VisionLEDMode.kOn);
      missalignment = this.checkTargets(missalignment);
    } else {
      // this.camera.setLED(VisionLEDMode.kOff);
    }

    SmartDashboard.putNumber("Missalignment speed PID", missalignment);

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(),
                rotation - missalignment,
                getYaw())
            // translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
  }

  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
    this.orientationWhenReleased = Rotation2d.fromDegrees(0);
  }

  public void resetHold() {
    this.orientationWhenReleased = getYaw();
  }

  public void setHold(double angle) {
    this.orientationWhenReleased = Rotation2d.fromDegrees(360 - angle);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getStates());
    this.field.setRobotPose(swerveOdometry.getPoseMeters());

    for (SwerveModule mod : mSwerveMods) {
      mod.updateDashboardValues();
    }
  }
}
