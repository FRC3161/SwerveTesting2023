package frc.lib.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  /* Module details */
  public int moduleNumber;

  /* Motors */
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  /* Encoders and their values */
  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;
  private double lastAngle;
  private double angleOffset;

  /* Controllers */
  public final SparkMaxPIDController driveController;
  public final SparkMaxPIDController angleController;
  public final PIDConstants anglePID;
  public final PIDConstants drivePID;
  public final SVAConstants driveSVA;
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

    this.setSpeed(desiredState, isOpenLoop);
    this.setAngle(desiredState);
  }

  public void resetToAbsolute() {
    double integratedAngleEncoderPosition = this.integratedAngleEncoder.getPosition();
    double absolutePosition = integratedAngleEncoderPosition - integratedAngleEncoderPosition % 360
        + (getCanCoder().getDegrees() - angleOffset);
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
    angleController.setFF(0);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    this.drivePID.applyPID(this.driveController);
    driveController.setFF(0);
    this.feedforward = driveSVA.getController();
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
      SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Drive Set Velocity",
          desiredState.speedMetersPerSecond);
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle
            .getDegrees();

    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = angle;
  }

  public void goToHome() {
    Rotation2d angle = getAngle();
    angleController.setReference(angle.getDegrees() - angle.getDegrees() % 360,
        ControlType.kPosition);
    lastAngle = angle.getDegrees() - angle.getDegrees() % 360;
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(this.angleEncoder.getAbsolutePosition());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(this.integratedAngleEncoder.getPosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(this.driveEncoder.getVelocity(), this.getAngle());
  }

  public SwerveModulePosition getPosition() {
    double distance = this.driveEncoder.getPosition() * Constants.Swerve.wheelCircumference;

    return new SwerveModulePosition(
      distance, this.getAngle()
    );
  }
}
