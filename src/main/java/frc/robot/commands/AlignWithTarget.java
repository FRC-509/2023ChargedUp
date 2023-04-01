package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.LimelightWrapper;
import frc.robot.vision.VisionTypes.PipelineState;
import frc.robot.vision.VisionTypes.TargetType;

public class AlignWithTarget extends SequentialCommandGroup {

	private class StrafeToMeetTarget extends CommandBase {
		private Swerve swerve;
		private LimelightWrapper limelight;
		private PIDController strafePID = new PIDController(0.1, 0.0, 0);
		private double targetAngle;
		private TargetType target;

		public StrafeToMeetTarget(Swerve swerve, LimelightWrapper limelight, TargetType target) {
			this.swerve = swerve;
			this.limelight = limelight;
			addRequirements(swerve);
		}

		@Override
		public void initialize() {
			switch (target) {
				case ConeNode:
					limelight.setPipeline(PipelineState.RetroReflective);
					targetAngle = Constants.Vision.angleOffsetHighGoal;
					break;
				case CubeNode:
					limelight.setPipeline(PipelineState.AprilTags);
					targetAngle = Constants.Vision.angleOffsetHighGoal;
					break;
				case Substation:
					limelight.setPipeline(PipelineState.MLGamePieces);
					targetAngle = Constants.Vision.angleOffsetHighGoal;
					break;
				default:
					break;
			}
			strafePID.setSetpoint(targetAngle);
			strafePID.setTolerance(0);
		}

		@Override
		public void execute() {
			if (!limelight.hasTarget()) {
				return;
			}
			// Strafe using a PID on the limelight's X offset to bring it to zero.
			double strafeOutput = MathUtil.clamp(strafePID.calculate(limelight.getXOffset()), -0.25, 0.25);
			swerve.drive(new Translation2d(0, -strafeOutput).times(Constants.maxSpeed), 0, true, false);
			SmartDashboard.putNumber("Strafing at speed:", strafeOutput * Constants.maxSpeed);
		}

		@Override
		public boolean isFinished() {
			SmartDashboard.putBoolean("Are we done?", strafePID.atSetpoint());
			return strafePID.atSetpoint();
		}

		@Override
		public void end(boolean wasInterrupted) {
			swerve.drive(new Translation2d(0, 0), 0, true, false);
		}
	}

	public AlignWithTarget(Swerve swerve, LimelightWrapper limelight, TargetType target) {
		swerve.setTargetHeading(0);
		addCommands(new StrafeToMeetTarget(swerve, limelight, target),
				new DriveCommand(swerve, 0.1, 0, 0, false).withTimeout(0.25),
				new DriveCommand(swerve, 0.0, 0, 0, false));
	}

}
