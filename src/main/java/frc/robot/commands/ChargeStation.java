
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeStation extends CommandBase {

	private PIDController pid = new PIDController(0.03, 0, 0);

	// Pitch buffer
	private double pitchBuffer;
	private Swerve swerve;
	private Pigeon2 gyro;
	private double invert;

	// Constructor
	public ChargeStation(Swerve swerve, Pigeon2 gyro, double invert) {
		this.swerve = swerve;
		this.gyro = gyro;
		this.invert = invert;
		pitchBuffer = 4;
		addRequirements(this.swerve);
	}

	@Override
	public void initialize() {

		pid.setSetpoint(0);

		Translation2d driveTranslation = new Translation2d(invert * 0.4, 0).times(Constants.maxSpeed);
		swerve.drive(driveTranslation, 0, false, true);
		Timer.delay(2);
		swerve.drive(new Translation2d(), 0, false, true);
	}

	// Execute
	@Override
	public void execute() {
		// Get gyro pitch change from last frame
		double increment = invert * MathUtil.clamp(pid.calculate(gyro.getPitch()), -0.4, 0.4);
		Translation2d driveTranslation = new Translation2d(increment, 0);
		System.out.println("Driving at speed: " + increment);
		swerve.drive(driveTranslation, 0, false, true);
	}

	// Is Finished
	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean wasInterrupted) {
	}
}