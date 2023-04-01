package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Led.BlinkinLedMode;
import frc.robot.util.telemetry.Thunderstorm;
import frc.robot.vision.VisionTypes.PipelineState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static boolean hasInitialized = false;

	private Command autonomousCommand;
	private RobotContainer robotContainer;
	private Thunderstorm thunderstorm;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		this.robotContainer = new RobotContainer();
		this.thunderstorm = new Thunderstorm();

		if (!hasInitialized) {
			robotContainer.armSubsystem.onFirstInit();
		}
		hasInitialized = true;
		this.robotContainer.limelight.setPipeline(PipelineState.RetroReflective);
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		SmartDashboard.putBoolean("Limelight has target", this.robotContainer.limelight.hasTarget());
		thunderstorm.update(this.robotContainer);

		switch (this.robotContainer.limelight.getPipeline()) {
			case AprilTags:
				SmartDashboard.putString("Limelight Pipeline", "AprilTags");
				break;
			case MLGamePieces:
				SmartDashboard.putString("Limelight Pipeline", "GamePieceML");
				break;
			case RetroReflective:
				SmartDashboard.putString("Limelight Pipeline", "RetroReflective");
				break;
			default:
				break;

		}
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
		robotContainer.armSubsystem.setPivotToEncoderValue();
		robotContainer.swerveSubsystem.setHeadingToGyro();

		switch (DriverStation.getAlliance()) {
			case Blue:
				Led.setMode(BlinkinLedMode.SOLID_BLUE);
				break;
			case Invalid:
				Led.setMode(BlinkinLedMode.SOLID_RED_ORANGE);
				break;
			case Red:
				Led.setMode(BlinkinLedMode.SOLID_RED);
				break;
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousInit() {
		switch (DriverStation.getAlliance()) {
			case Blue:
				Led.setMode(BlinkinLedMode.SOLID_BLUE);
				break;
			case Invalid:
				Led.setMode(BlinkinLedMode.SOLID_RED_ORANGE);
				break;
			case Red:
				Led.setMode(BlinkinLedMode.SOLID_RED);
				break;
		}

		this.autonomousCommand = this.robotContainer.getAutonomousCommand();
		if (autonomousCommand != null) {
			this.autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		switch (DriverStation.getAlliance()) {
			case Blue:
				Led.setMode(BlinkinLedMode.SOLID_BLUE);
				break;
			case Invalid:
				Led.setMode(BlinkinLedMode.SOLID_RED_ORANGE);
				break;
			case Red:
				Led.setMode(BlinkinLedMode.SOLID_RED);
				break;
		}
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (this.autonomousCommand != null) {
			this.autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		switch (DriverStation.getAlliance()) {
			case Blue:
				Led.setMode(BlinkinLedMode.SOLID_BLUE);
				break;
			case Invalid:
				Led.setMode(BlinkinLedMode.SOLID_RED_ORANGE);
				break;
			case Red:
				Led.setMode(BlinkinLedMode.SOLID_RED);
				break;
		}

		if (RobotController.isBrownedOut()) {
			Led.setMode(BlinkinLedMode.SOLID_HOT_PINK);
		}
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
