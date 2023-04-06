package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Led.BlinkinLedMode;
import frc.robot.util.math.Utils;
import frc.robot.vision.LimelightWrapper;
import frc.robot.vision.VisionTypes.TargetType;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
	private Swerve s_Swerve;
	private LimelightWrapper limelight;
	private DoubleSupplier translationSup;
	private DoubleSupplier strafeSup;
	private DoubleSupplier rotationSup;
	private BooleanSupplier robotCentricSup;
	private BooleanSupplier xStanceSup;
	private BooleanSupplier faceForward;
	private BooleanSupplier faceBackward;
	private BooleanSupplier lockToTarget;
	private boolean isTeleop;

	public DriveCommand(
			Swerve s_Swerve,
			LimelightWrapper limelight,
			DoubleSupplier translationSup,
			DoubleSupplier strafeSup,
			DoubleSupplier rotationSup,
			BooleanSupplier robotCentricSup,
			BooleanSupplier xStanceSup,
			BooleanSupplier faceForward,
			BooleanSupplier faceBackward,
			BooleanSupplier lockToTarget) {
		this.s_Swerve = s_Swerve;
		this.limelight = limelight;
		addRequirements(s_Swerve);

		this.translationSup = translationSup;
		this.strafeSup = strafeSup;
		this.rotationSup = rotationSup;
		this.robotCentricSup = robotCentricSup;
		this.xStanceSup = xStanceSup;
		this.faceForward = faceForward;
		this.faceBackward = faceBackward;
		this.lockToTarget = lockToTarget;
		isTeleop = true;
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
		this.xStanceSup = () -> false;
		this.faceForward = () -> false;
		this.faceBackward = () -> false;
		this.lockToTarget = () -> false;
		isTeleop = false;
	}

	@Override
	public void execute() {
		/* Get Values, Deadband */
		double translationVal = MathUtil.applyDeadband(this.translationSup.getAsDouble(), Constants.stickDeadband);
		double strafeVal = MathUtil.applyDeadband(this.strafeSup.getAsDouble(), Constants.stickDeadband);
		double rotationVal = MathUtil.applyDeadband(this.rotationSup.getAsDouble(), Constants.stickDeadband);

		if (lockToTarget.getAsBoolean()) {
			if (!limelight.hasTarget()) {
				RobotContainer.ledMode = BlinkinLedMode.SOLID_HOT_PINK;
			} else {
				// s_Swerve.setTargetHeading(0.0);
				RobotContainer.ledMode = BlinkinLedMode.SOLID_LAWN_GREEN;
				(new AlignWithTarget(s_Swerve, limelight, translationSup, TargetType.ConeNode,
						lockToTarget)).schedule();
				return;
			}
		} else {
			RobotContainer.setLedToAllianceColors();
		}

		/* Drive */
		if (xStanceSup.getAsBoolean()) {
			s_Swerve.enterXStance();
		} else if (faceForward.getAsBoolean() || faceBackward.getAsBoolean()) {

			if (faceBackward.getAsBoolean()) {
				s_Swerve.setTargetHeading(0.0d);
			} else {
				s_Swerve.setTargetHeading(180.0d);
			}

			this.s_Swerve.drive(
					new Translation2d(translationVal, strafeVal).times(0.4 * Constants.maxSpeed),
					0.0d,
					!this.robotCentricSup.getAsBoolean(),
					false);
		} else {
			this.s_Swerve.drive(
					new Translation2d(translationVal, strafeVal).times(Constants.maxSpeed),
					rotationVal * Constants.maxAngularVelocity,
					!this.robotCentricSup.getAsBoolean(),
					false);
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		s_Swerve.stopModules();
	}
}