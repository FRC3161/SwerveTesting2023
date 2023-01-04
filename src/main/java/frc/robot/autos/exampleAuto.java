package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import org.photonvision.common.hardware.VisionLEDMode;

public class exampleAuto extends SequentialCommandGroup {
        public exampleAuto(Swerve s_Swerve) {
                ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("test",
                                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                HashMap<String, Command> eventMap = new HashMap<>();

                SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(s_Swerve::getPose, s_Swerve::resetOdometry,
                                Constants.Swerve.swerveKinematics, new PIDConstants(0.3, 0, 0.05),
                                new PIDConstants(0.2, 0, 0.05), s_Swerve::setModuleStates,
                                eventMap,
                                s_Swerve);

                addCommands(new InstantCommand(() -> s_Swerve.camera.setLED(VisionLEDMode.kOn)),
                                autoBuilder.fullAuto(pathGroup),
                                new InstantCommand(() -> s_Swerve.camera.setLED(VisionLEDMode.kOff)));
                // addCommands(new InstantCommand(() ->
                // s_Swerve.resetOdometry(pathGroup.get(0).getInitialHolonomicPose())),
                // autoBuilder.followPathWithEvents(pathGroup.get(0)),
                // new InstantCommand(() -> s_Swerve.camera.setLED(VisionLEDMode.kOn)),
                // autoBuilder.followPathWithEvents(pathGroup.get(1)),
                // new InstantCommand(() -> s_Swerve.camera.setLED(VisionLEDMode.kOff)));
        }
}
