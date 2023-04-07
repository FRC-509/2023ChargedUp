
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.drivers.PigeonWrapper;

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
	private PigeonWrapper gyro;
	private double invert;

	// Constructor
	public ChargeStation(Swerve swerve, PigeonWrapper gyro, boolean invert) {
		this.swerve = swerve;
		this.gyro = gyro;
		// invert determines what direction the robot will face during the command.
		// Pass -1.0 if the robot should approach the charge station while facing away
		// from it, and 1.0 if the robot should approach the charge station while facing
		// it.
		this.invert = invert ? -1 : 1;
		pitchBuffer = 4;
		addRequirements(this.swerve);
	}

	@Override
	public void initialize() {
		// Set the target setpoint to zero degrees.
		pid.setSetpoint(0);
		// Drive forward for two seconds at 40% of the maximum speed, then stop
		// (gets us onto the edge of the charge station, starting from the edge of the
		// community)
		Translation2d driveTranslation = new Translation2d(invert * 0.3, 0).times(Constants.maxSpeed);
		swerve.drive(driveTranslation, 0, false, true);
		Timer.delay(1.5);
		swerve.drive(new Translation2d(), 0, false, true);
	}

	@Override
	public void execute() {
		// Get the gyro pitch, pass it to the PID controller and clamp it to 40% speed
		// (prevents us from flying off the charge station at first)
		double increment = invert * MathUtil.clamp(pid.calculate(gyro.getPitch()), -0.5, 0.5);
		// Drive forward by the calculated increment
		Translation2d driveTranslation = new Translation2d(increment, 0);
		System.out.println("Driving at speed: " + increment);
		swerve.drive(driveTranslation, 0, false, true);
	}

	@Override
	public boolean isFinished() {
		// Ensure that the command is always running
		return false;
	}

	@Override
	public void end(boolean wasInterrupted) {
	}
}
