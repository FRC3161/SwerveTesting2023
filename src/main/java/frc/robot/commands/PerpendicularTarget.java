package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PerpendicularTarget extends CommandBase {
    private final Swerve swerveSubsystem;
    private final PIDController targetTranslationPID = Constants.Swerve.targetTranslationPID.getController();
    private final PIDController robotRotationPID = Constants.Swerve.robotRotationPID.getController();

    public PerpendicularTarget(Swerve swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.robotRotationPID.enableContinuousInput(-90, 90);
        this.robotRotationPID.setP(0.05);
        this.robotRotationPID.setTolerance(5);

        this.targetTranslationPID.setTolerance(0.1);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        PhotonPipelineResult results = this.swerveSubsystem.camera.getLatestResult();
        if (!results.hasTargets())
            return;

        List<PhotonTrackedTarget> targets = results.getTargets();
        PhotonTrackedTarget target = targets.get(0);

        int targetID = target.getFiducialId();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        SmartDashboard.putNumber("target id", targetID);
        SmartDashboard.putNumber("target x", bestCameraToTarget.getX());
        SmartDashboard.putNumber("target Y", bestCameraToTarget.getY());
        this.swerveSubsystem.orientationWhenReleased = this.swerveSubsystem.getYaw();

        SmartDashboard.putNumber("robotRotationPID target",
                this.robotRotationPID.calculate(bestCameraToTarget.getRotation().getZ() * (180 / Math.PI), 0));
        SmartDashboard.putNumber("targetTranslationPID target",
                this.targetTranslationPID.calculate(bestCameraToTarget.getY(), 0));
        if (this.robotRotationPID.atSetpoint()) {
            double rotation = -MathUtil.clamp(
                    this.robotRotationPID.calculate(bestCameraToTarget.getRotation().getZ() *
                            (180 / Math.PI),
                            0),
                    -1, 1);
            this.swerveSubsystem.drive(
                    new Translation2d(0,
                            -this.targetTranslationPID.calculate(bestCameraToTarget.getY(), 0)),
                    rotation < 0.15 && rotation > -0.15 ? 0 : rotation,
                    true, true, false);
        } else {
            this.swerveSubsystem.drive(
                    new Translation2d(0, 0),
                    -MathUtil.clamp(
                            this.robotRotationPID.calculate(bestCameraToTarget.getRotation().getZ() *
                                    (180 / Math.PI),
                                    0),
                            -1, 1),
                    true, true, false);
        }

    }

}
