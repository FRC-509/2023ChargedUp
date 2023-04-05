package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Led.BlinkinLedMode;
import frc.robot.util.PIDWrapper;
import frc.robot.vision.LimelightWrapper;
import frc.robot.vision.VisionTypes.PipelineState;
import frc.robot.vision.VisionTypes.TargetType;

public class AlignWithTarget extends CommandBase {

	private Swerve swerve;
	private LimelightWrapper limelight;
	private PIDWrapper strafePID = new PIDWrapper(0.125d, 0.0001d, 0.000001d, 0.0d);
	private double targetAngle;
	private TargetType target;
	private DoubleSupplier forwardStrafe;
	private BooleanSupplier run;

	public AlignWithTarget(Swerve swerve, LimelightWrapper limelight, DoubleSupplier forwardStrafe,
			TargetType target, BooleanSupplier run) {

		limelight.setPipeline(PipelineState.RetroReflective);

		this.swerve = swerve;
		this.limelight = limelight;
		this.target = target;
		this.forwardStrafe = forwardStrafe;
		this.run = run;
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		if (!limelight.hasTarget()) {
			end(true);
		}

		switch (target) {
			case ConeNode:
				limelight.setPipeline(PipelineState.RetroReflective);
				targetAngle = Constants.Vision.highConeTargetAngle;
				break;
			case CubeNode:
				limelight.setPipeline(PipelineState.AprilTags);
				targetAngle = Constants.Vision.highConeTargetAngle;
				break;
			case Substation:
				limelight.setPipeline(PipelineState.MLGamePieces);
				targetAngle = Constants.Vision.substationTargetAngle;
				break;
			default:
				break;
		}
		strafePID.setSetpoint(targetAngle);
		strafePID.setTolerance(0.05);
	}

	@Override
	public void execute() {
		if (strafePID.atSetpoint()) {
			RobotContainer.ledMode = BlinkinLedMode.FIXED_CONFETTI;
		}

		if (!limelight.hasTarget()) {
			return;
		}

		// SmartDashboard.putNumber("limelight offset: ", limelight.getXOffset());
		// strafePID.debug("strafe");

		// Strafe using a PID on the limelight's X offset to bring it to zero.
		double strafeOutput = strafePID.calculate(limelight.getXOffset());
		swerve.drive(new Translation2d(forwardStrafe.getAsDouble(), strafeOutput).times(0.4d * Constants.maxSpeed),
				0,
				true, false);
	}

	@Override
	public boolean isFinished() {
		RobotContainer.setLedToAllianceColors();
		return !run.getAsBoolean();
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
	}
}
