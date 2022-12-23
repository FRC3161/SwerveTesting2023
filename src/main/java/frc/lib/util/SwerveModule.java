package frc.lib.util;

import java.util.Map;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private double lastAngle;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  public final SparkMaxPIDController driveController;
  public final SparkMaxPIDController angleController;
  public final PIDConstants anglePID;
  public final PIDConstants drivePID;
  public final SVAConstants driveSVA;
  private boolean hasBeenReset = false;

  public SimpleMotorFeedforward feedforward;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;
    this.anglePID = moduleConstants.anglePID;
    this.drivePID = moduleConstants.drivePID;
    this.driveSVA = moduleConstants.driveSVA;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID, moduleConstants.cancoderCANBUS);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();

    // ANGLE PID DASHBOARD
    this.anglePID.sendDashboard(Constants.Swerve.moduleNames[this.moduleNumber] + "Angle");
    // DRIVE PID DASHBOARD
    this.drivePID.sendDashboard(Constants.Swerve.moduleNames[this.moduleNumber] + "Drive");
    // DRIVE SVA DASHBOARD
    this.driveSVA.sendDashboard(Constants.Swerve.moduleNames[this.moduleNumber] + "Drive");

    this.updateDashboardValues();
  }

  public void updateDashboardValues() {
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Integrated Encoder",
        this.getState().angle.getDegrees());
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Can Coder",
        this.getCanCoder().getDegrees());
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Set Point", this.lastAngle);
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Drive Encoder Velocity",
        this.driveEncoder.getVelocity());
  }

  public void updateControllerValues() {
    this.feedforward = this.driveSVA.retrieveDashboard();
    this.anglePID.retrieveDashboard(this.angleController);
    this.drivePID.retrieveDashboard(this.driveController);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    this.updateControllerValues();
    desiredState = OnboardModuleState.optimize(
        desiredState,
        getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      double power = feedforward.calculate(desiredState.speedMetersPerSecond);
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          power);
      SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Drive Set Velocity",
          desiredState.speedMetersPerSecond);
    }

    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle
            .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
    // // Jittering.
    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = angle;
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset;
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kSensorDataOnly);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    this.anglePID.applyPID(this.angleController);
    angleController.setFF(Constants.Swerve.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    this.drivePID.applyPID(this.driveController);
    driveController.setFF(Constants.Swerve.driveKFF);
    this.feedforward = driveSVA.getController();
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveEncoder.setPosition(0.0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d angle = Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    return new SwerveModuleState(velocity, angle);
  }
}
