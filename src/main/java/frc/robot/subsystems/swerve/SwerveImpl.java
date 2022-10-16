package frc.robot.subsystems.swerve;

import java.util.List;

import frc.robot.Constants;

public class SwerveImpl {

	/**
	 * Swerve modules
	 */
	public List<SwerveModule> swerveModules;

	public SwerveImpl() {
		this.swerveModules = SwerveModule.loadModules(Constants.serveModules);
	}
}
