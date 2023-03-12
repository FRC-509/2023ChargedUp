package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
	private Swerve s_Swerve;
	private DoubleSupplier translationSup;
	private DoubleSupplier strafeSup;
	private DoubleSupplier rotationSup;
	private BooleanSupplier robotCentricSup;
	private BooleanSupplier xStanceSup;

	public DriveCommand(
			Swerve s_Swerve,
			DoubleSupplier translationSup,
			DoubleSupplier strafeSup,
			DoubleSupplier rotationSup,
			BooleanSupplier robotCentricSup,
			BooleanSupplier xStanceSup) {
		this.s_Swerve = s_Swerve;
		addRequirements(s_Swerve);

		this.translationSup = translationSup;
		this.strafeSup = strafeSup;
		this.rotationSup = rotationSup;
		this.robotCentricSup = robotCentricSup;
		this.xStanceSup = xStanceSup;
	}

	public DriveCommand(
			Swerve s_Swerve,
			double translation,
			double strafe,
			double rotation,
			boolean robotCentric) {
		this.s_Swerve = s_Swerve;
		addRequirements(s_Swerve);

		this.translationSup = () -> translation;
		this.strafeSup = () -> strafe;
		this.rotationSup = () -> rotation;
		this.robotCentricSup = () -> robotCentric;
	}

	@Override
	public void execute() {
		/* Get Values, Deadband */
		double translationVal = MathUtil.applyDeadband(this.translationSup.getAsDouble(), Constants.stickDeadband);
		double strafeVal = MathUtil.applyDeadband(this.strafeSup.getAsDouble(), Constants.stickDeadband);
		double rotationVal = MathUtil.applyDeadband(this.rotationSup.getAsDouble(), Constants.stickDeadband);

		/* Drive */
		if (xStanceSup.getAsBoolean()) {
			this.s_Swerve.enterXStance();
		} else {
			this.s_Swerve.drive(
					new Translation2d(translationVal, strafeVal).times(Constants.maxSpeed),
					rotationVal * Constants.maxAngularVelocity,
					!this.robotCentricSup.getAsBoolean(), false);
		}
	}
}