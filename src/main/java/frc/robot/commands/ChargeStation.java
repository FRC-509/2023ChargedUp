
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeStation extends CommandBase {

	private PIDController pid = new PIDController(0.01, 0, 0);

	// Pitch buffer
	private double pitchBuffer;
	private Swerve swerve;
	private Pigeon2 gyro;

	// Constructor
	public ChargeStation(Swerve swerve, Pigeon2 gyro) {
		this.swerve = swerve;
		this.gyro = gyro;
		pitchBuffer = 5;
		addRequirements(this.swerve);
	}

	@Override
	public void initialize() {

		pid.setSetpoint(0);

		Translation2d driveTranslation = new Translation2d(0.5, 0).times(Constants.maxSpeed);
		swerve.drive(driveTranslation, 0, true);
		Timer.delay(1.0);
		swerve.drive(new Translation2d(), 0, false);
	}

	// Execute
	@Override
	public void execute() {

		// Get gyro pitch change from last frame
		double increment = pid.calculate(gyro.getPitch());
		Translation2d driveTranslation = new Translation2d(increment, 0);
		swerve.drive(driveTranslation, 0, true);
	}

	// Is Finished
	@Override
	public boolean isFinished() {
		return Math.abs(gyro.getPitch()) <= pitchBuffer;
	}

	@Override
	public void end(boolean wasInterrupted) {
		// Enter the drivetrain's X-Stance to lock our position on the station.
		swerve.enterXStance();
	}
}