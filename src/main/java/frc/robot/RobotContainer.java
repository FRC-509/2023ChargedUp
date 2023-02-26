package frc.robot;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Led.PatternID;
import frc.robot.util.controllers.JoystickController;
import frc.robot.util.controllers.LogitechController;
import frc.robot.util.controllers.JoystickController.StickButton;
import frc.robot.util.controllers.LogitechController.LogiButton;
import frc.robot.vision.*;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	public final JoystickController leftStick = new JoystickController(1);
	public final JoystickController rightStick = new JoystickController(0);
	public final LogitechController operatorController = new LogitechController(2);

	public final LimelightWrapper limelight = new LimelightWrapper(Constants.limelightName);
	public final Pigeon2 pigeon2 = new Pigeon2(30, Constants.CANIvore);

	public final Swerve swerveSubsystem;
	public final Arm armSubsystem;
	public final Claw clawSubsystem;

	private final SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		// Initialize and configure the gyroscope.
		this.pigeon2.configFactoryDefault();
		this.zeroGyro();

		// Initialize subsystems.
		this.swerveSubsystem = new Swerve(pigeon2, limelight);
		this.armSubsystem = new Arm();
		this.clawSubsystem = new Claw();

		// Configure button bindings and put our sendable chooser on the dashboard.
		this.configureButtonBindings();
		this.addAutonomousRoutines();
	}

	public void configureButtonBindings() {
		// Set the default command of the drive train subsystem to DriveCommand.
		swerveSubsystem.setDefaultCommand(new DriveCommand(
				swerveSubsystem,
				() -> -leftStick.getY(),
				() -> -leftStick.getX(),
				() -> -rightStick.getX(),
				() -> leftStick.getRawButton(2)));
		clawSubsystem
				.setDefaultCommand(new ClawCommand(clawSubsystem, () -> operatorController.isPressed(LogiButton.A)));
		armSubsystem.setDefaultCommand(
				new ArmCommand(armSubsystem,
						() -> operatorController.getLeftStickX() * Constants.armPivotOperatorCoefficient,
						() -> operatorController.getRightStickY() * -Constants.armExtensionOperatorCoefficient));

		// swerveSubsystem.setDefaultCommand(new Tuning(swerveSubsystem));

		leftStick.isDownBind(StickButton.Bottom, new InstantCommand(() -> zeroGyro(), swerveSubsystem));

		operatorController.isPressedBind(LogiButton.X, new InstantCommand(() -> Led.set(PatternID.OFF)));
		operatorController.isPressedBind(LogiButton.Y, new InstantCommand(() -> Led.set(PatternID.YELLOW)));
		operatorController.isPressedBind(LogiButton.B, new InstantCommand(() -> Led.set(PatternID.VIOLET)));
	}

	private void addAutonomousRoutines() {
		chooser.setDefaultOption("WeekZeroTaxi", new DriveCommand(
				swerveSubsystem,
				1.0,
				0,
				0, true)
				.withTimeout(0.7));
		chooser.addOption("None", null);
		SmartDashboard.putData(chooser);
	}

	public void zeroGyro() {
		this.pigeon2.setYaw(0);
		this.pigeon2.zeroGyroBiasNow();
	}

	public Pose2d getEstimatedPose() {
		return swerveSubsystem.getPose();
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
