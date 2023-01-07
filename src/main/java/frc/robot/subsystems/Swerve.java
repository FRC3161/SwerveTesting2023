package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import org.photonvision.PhotonCamera;
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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;

/**
 * Swerve subsystem
 *
 * @param swerveOdometry
 * @param mSwerveMods             Swerve modules constants
 * @param gyro                    The gyroscope
 * @param orientationWhenReleased The orientation of the robot when the rotation
 *                                axis' value is 0
 * @param rotationControllerSpeed the tmp value to check determine when the
 *                                rotation value becomes 0
 * @param robotRotationPID        PID controller for position hold
 * @param targetRotationPID       PID controller for vision
 * @param camera
 */
public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;
  public Rotation2d orientationWhenReleased;
  private double rotationControllerSpeed = 0.0;
  public final PIDController robotRotationPID;
  private final PIDController targetRotationPID;
  public final PhotonCamera camera = new PhotonCamera(Constants.Vision.cameraName);

  public Swerve() {
    /* Gyro setup */
    gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.pigeonCanBUS);
    gyro.configFactoryDefault();
    zeroGyro();

    /* Custom PID controllers setup */
    this.robotRotationPID = Constants.Swerve.robotRotationPID.getController();
    this.robotRotationPID.enableContinuousInput(-180, 180);

    this.targetRotationPID = Constants.Swerve.targetRotationPID.getController();

    /* Swerve modules setup */
    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants),
      new SwerveModule(1, Constants.Swerve.Mod1.constants),
      new SwerveModule(2, Constants.Swerve.Mod2.constants),
      new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), this.getPositions());
  }

  /**
   * Updates the robots position hold target when the rotation axis is not being
   * used
   * 
   * @param rotation the value of the rotation axis
   */
  public void rotationUpdate(double rotation) {
    if (this.rotationControllerSpeed != 0.0) {
      this.orientationWhenReleased = getYaw();
    }
    this.rotationControllerSpeed = rotation;
  }

  /**
   * Checks for any targets from the camera and if there was any, it would return
   * the new missAlignment value for the rotation of the robot
   * 
   * @param missAlignment
   * @return A new value for missAlignment if there is any targets
   */
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

      return this.targetRotationPID.calculate(-bestCameraToTarget.getY(), 0);
    }

    return missAlignment;
  }

  /**
   * The main function used for driving the robot
   * 
   * @param translation
   * @param rotation
   * @param fieldRelative
   * @param isOpenLoop    True -> no PID, False -> PID
   * @param shoot
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
      boolean shoot) {
    this.rotationUpdate(rotation);

    // Clamp the output to the maxSpeed so that the robot doesn't make hole in the
    // wall :D
    double missalignment = MathUtil.clamp(
        this.robotRotationPID.calculate(getYaw().getDegrees() % 360,
            this.orientationWhenReleased.getDegrees() % 360),
        -Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed);

    // Shoot? YES :D, check for targets and then reassign the missAlignment value if
    // we
    // have to turn to the target!
    if (shoot) {
      // this.camera.setLED(VisionLEDMode.kOn);
      missalignment = this.checkTargets(missalignment);
    } else {
      // this.camera.setLED(VisionLEDMode.kOff);
    }

    SmartDashboard.putNumber("Missalignment speed PID", missalignment);

    // get the states for each module
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(),
                rotation + (Constants.Swerve.driveInvert ? missalignment : -missalignment),
                getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    // Get rid of tiny tiny movements in the wheels to have more consistent driving
    // experience
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // set the states for each module
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    var commandGroup = new SequentialCommandGroup();

    commandGroup.addCommands(new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      if (isFirstPath) {
        this.resetOdometry(traj.getInitialHolonomicPose());
      }
    }),
        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            Constants.AutoConstants.translationPID.getController(),
            Constants.AutoConstants.translationPID.getController(),
            Constants.AutoConstants.rotationPID.getController(),
            this::setModuleStates, // Module states consumer
            this // Requires this drive subsystem
        ));
    return commandGroup;
  }

  /**
   * The function used in auto to set the states for each module, does not have
   * vision tracking and position hold.
   * 
   * @param desiredStates the desired state foe each module
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * Function used in auto to get an estimate of where the bot is on the field in
   * auto
   * 
   * @return The position of the robot in
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * set swerveOdometry of the robot when requested to the desired position
   * 
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  /** Reset the module encoder values */
  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Get the current state of the modules
   * 
   * @return state of the modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  /** Reset the gyro and position hold */
  public void zeroGyro() {
    gyro.setYaw(0);
    this.orientationWhenReleased = Rotation2d.fromDegrees(0);
  }

  /** reset position hold value to whatever the gyro is */
  public void resetHold() {
    this.orientationWhenReleased = getYaw();
  }

  /**
   * Set the positionHold value to whatever angle is given
   * 
   * @param angle the angle to point the robot towards and hold
   */
  public void setHold(double angle) {
    this.orientationWhenReleased = Rotation2d.fromDegrees(360 - angle);
  }

  /**
   * get the orientation of the robot
   * 
   * @return the orientation of the robot
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    SmartDashboard.putNumber("yaw", getYaw().getDegrees());

    for (SwerveModule mod : mSwerveMods) {
      mod.updateDashboardValues();
    }
  }
}
