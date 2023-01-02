package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotModes;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private DoubleSupplier POVSup;
  private BooleanSupplier rightBumper;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Constants.Swerve.translationChangeLimit);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.Swerve.strafeChangeLimit);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.Swerve.rotationChangeLimit);

  /**
   * The default command for Swerve and which is being used for driving, if any
   * other command overrides this one,
   * all the driving abilities will be suspended until it's back on the command
   * scheduler
   * 
   * @param swerve          An instance of the swerve subsustem
   * @param translationSup  the value of translation axis on the controller
   * @param strafeSup       the value of starfe axis on the controller
   * @param rotationSup     the rotation value
   * @param robotCentricSup whether or not the robot is driving relative to the
   *                        field or relative to itself
   * @param POVSup          the D Pad on the controller
   * @param rightBumper     whether or not the rightBumper is held down
   */
  public TeleopSwerve(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      DoubleSupplier POVSup,
      BooleanSupplier rightBumper) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.POVSup = POVSup;
    this.rightBumper = rightBumper;
  }

  @Override
  public void initialize() {
    // Set the hold position to the current orientation of the robot
    this.swerve.resetHold();
  }

  @Override
  public void execute() {
    double translationVal, strafeVal, rotationVal;

    /* Get Values, Deadband */
    switch (Constants.Operators.driverMode) {
      case Raw:
        translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
        strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
        rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);
        break;

      case Slew:
        translationVal = translationLimiter
            .calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
        strafeVal = strafeLimiter
            .calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
        rotationVal = rotationLimiter
            .calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));
        break;
      default:
        translationVal = strafeVal = rotationVal = 0;
        break;
    }

    if (Constants.robotMode == RobotModes.Testing) {
      translationVal *= 0.2;
      strafeVal *= 0.2;
      rotationVal *= 0.2;
    }

    double POVVal = this.POVSup.getAsDouble();

    if (POVVal != -1) {
      // set the hold position of the robot to whatever orientation is chosen by the
      // driver
      this.swerve.setHold(POVVal);
    }

    /* Drive */
    swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true, // True -> driving based on percent output, False -> driving based on PID,
              // FeedForward
        this.rightBumper.getAsBoolean());
  }
}
