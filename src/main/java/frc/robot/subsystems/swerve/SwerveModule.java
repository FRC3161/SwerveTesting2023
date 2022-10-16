package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.SwerveModuleConstants;
import frc.robot.motors.AbsoluteMotor;
import frc.robot.motors.RelativeMotor;

public class SwerveModule {
    public int number;
    RelativeMotor wheel;
    AbsoluteMotor pod;

    public SwerveModule(int number, RelativeMotor wheel, AbsoluteMotor pod) {
        this.number = number;
        this.wheel = wheel;
        this.pod = pod;
    }

    public SwerveModule(int number, SwerveModuleConstants constants) {
        this.number = number;
        this.wheel = new RelativeMotor(new CANSparkMax(constants.wheelController, MotorType.kBrushless),
                constants.wheelPID);
        this.pod = new AbsoluteMotor(new CANSparkMax(constants.podController, MotorType.kBrushless), constants.podPID,
                new WPI_CANCoder(constants.podEncoder));
    }

    public static List<SwerveModule> loadModules(SwerveModuleConstants[] constants) {
        List<SwerveModule> output = new ArrayList<SwerveModule>();
        for (int i = 0; i < constants.length; i++) {
            output.add(new SwerveModule(i, constants[i]));
        }

        return output;
    }
}