package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.constants.SwerveModuleConstants;
import frc.robot.motors.PodMotor;
import frc.robot.motors.WheelMotor;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public int number;
    private double angleOffset;
    private double lastAngle;

    WheelMotor wheel;
    PodMotor pod;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    public SwerveModule(
            int number, WheelMotor wheel,
            PodMotor pod,
            double angleOffset) {
        this.number = number;
        this.wheel = wheel;
        this.pod = pod;
        this.angleOffset = angleOffset;
        this.lastAngle = this.getState().angle.getDegrees();
    }

    public SwerveModule(int number, SwerveModuleConstants constants) {
        this.number = number;
        this.wheel = new WheelMotor(new CANSparkMax(constants.wheelController, MotorType.kBrushless),
                constants.wheelPID);
        this.pod = new PodMotor(new CANSparkMax(constants.podController, MotorType.kBrushless), constants.podPID,
                new WPI_CANCoder(constants.podEncoder));
        this.angleOffset = constants.angleOffset;
    }

    public static List<SwerveModule> loadModules(SwerveModuleConstants[] constants) {
        List<SwerveModule> output = new ArrayList<SwerveModule>();
        for (int i = 0; i < constants.length; i++) {
            output.add(new SwerveModule(i, constants[i]));
        }

        return output;
    }

    public SwerveModuleState getState() {
        double velocity = this.wheel.encoder.getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(this.pod.externalEncoder.getPosition());
        return new SwerveModuleState(velocity, angle);
    }
}
