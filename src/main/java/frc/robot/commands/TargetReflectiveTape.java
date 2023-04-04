package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.LimelightWrapper;
import frc.robot.vision.VisionTypes.PipelineState;

public class TargetReflectiveTape extends SequentialCommandGroup {

	private class StrafeToMeetTarget extends CommandBase {
		private Swerve swerve;
		private LimelightWrapper limelight;
		private PIDController strafePID = new PIDController(0.0, 0, 0);
		private double targetAngle = 4.2d;

		public StrafeToMeetTarget(Swerve swerve, LimelightWrapper limelight) {
			this.swerve = swerve;
			this.limelight = limelight;
			addRequirements(swerve);
		}

		@Override
		public void initialize() {
			limelight.setPipeline(PipelineState.RetroReflective);
			swerve.setTargetHeading(180.0d);
			strafePID.setSetpoint(targetAngle);
			strafePID.setTolerance(2.5);
		}

		@Override
		public void execute() {
			// Strafe using a PID on the limelight's X offset to bring it to zero.
			double strafeOutput = strafePID.calculate(limelight.getXOffset());
			swerve.drive(new Translation2d(strafeOutput, 0).times(Constants.maxSpeed), 0, true, false);
			SmartDashboard.putNumber("Strafing at speed:", strafeOutput * Constants.maxSpeed);
		}

		@Override
		public boolean isFinished() {
			SmartDashboard.putBoolean("Are we done?", strafePID.atSetpoint() || limelight.hasTarget());
			return strafePID.atSetpoint() || limelight.hasTarget();
		}

		@Override
		public void end(boolean wasInterrupted) {
			swerve.drive(new Translation2d(0, 0), 0, true, false);
		}
	}

	private Swerve swerve;
	private LimelightWrapper limelight;

	public TargetReflectiveTape(Swerve swerve, LimelightWrapper limelight) {
		this.swerve = swerve;
		this.limelight = limelight;
		swerve.setTargetHeading(0);
		addCommands(new StrafeToMeetTarget(swerve, limelight),
				new DriveCommand(swerve, 0.1, 0, 0, false).withTimeout(0.25),
				new DriveCommand(swerve, 0.0, 0, 0, false));
	}

}
