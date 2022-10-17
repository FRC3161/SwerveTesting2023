package frc.robot.subsystems.swerve;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveImpl extends SubsystemBase {

	/**
	 * Swerve modules
	 */
	public List<SwerveModule> swerveModules;

	public SwerveImpl() {
		this.swerveModules = SwerveModule.loadModules(Constants.swerveModules);
	}

	@Override
	public void periodic() {

	}
}
