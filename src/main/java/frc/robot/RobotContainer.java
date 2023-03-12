package frc.robot;

import frc.robot.autonomous.OneConeAndTaxiStable;
import frc.robot.autonomous.PickUpCubeFromGround;
import frc.robot.autonomous.OneConeAndChargeStation;
import frc.robot.autonomous.OneConeAndTaxiPP;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
//import frc.robot.subsystems.Led;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.cameraserver.*;
import frc.robot.subsystems.TimeStamp;
//import frc.robot.subsystems.Led.PatternID;
import frc.robot.util.controllers.JoystickController;
import frc.robot.util.controllers.LogitechController;
import frc.robot.util.controllers.JoystickController.StickButton;
import frc.robot.util.controllers.LogitechController.LogiButton;
import frc.robot.vision.*;
import frc.robot.subsystems.Led;
import java.io.IOException;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
	public final LogitechController controller = new LogitechController(2);

	public static TimeStamp timeStamp = new TimeStamp();

	public final LimelightWrapper limelight = new LimelightWrapper(Constants.limelightName);
	public final Pigeon2 pigeon2 = new Pigeon2(30, Constants.CANIvore);
	public AprilTagFieldLayout fieldLayout;

	public final Swerve swerveSubsystem;
	public final Arm armSubsystem;
	public final Claw clawSubsystem;
	public final UsbCamera usbCamera = new UsbCamera("509cam", 1);

	// public final Spark led;
	private final SendableChooser<Command> chooser = new SendableChooser<Command>();
	private final SendableChooser<String> loopTypeForExtension = new SendableChooser<String>();

	public RobotContainer() {
		UsbCamera cam = CameraServer.startAutomaticCapture();
		cam.setFPS(15);
		cam.setResolution(10, 10);

		// SmartDashboard.putData(usbCamera.get);
		// Initialize and configure the gyroscope.
		this.pigeon2.configFactoryDefault();
		this.pigeon2.configMountPoseYaw(180);
		// Initialize subsystems.
		this.swerveSubsystem = new Swerve(timeStamp, pigeon2, limelight);
		this.armSubsystem = new Arm();
		this.clawSubsystem = new Claw();

		// Configure button bindings
		this.configureButtonBindings();
		this.addAutonomousRoutines();

		this.loopTypeForExtension.addOption("Closed Loop", "CL");
		this.loopTypeForExtension.addOption("Open Loop", "OL");
		SmartDashboard.putData(loopTypeForExtension);

		this.zeroGyro();

		// Initialize the AprilTagFieldLayout
		try {
			fieldLayout = new AprilTagFieldLayout(
					Filesystem
							.getDeployDirectory()
							.toPath()
							.resolve("2023-chargedup.json"));
		} catch (IOException e) {
			DriverStation.reportWarning("failed to load april tag field layout json", true);
			fieldLayout = null;
		}
	}

	public double deltaTime() {
		return timeStamp.deltaTime();
	}

	public void configureButtonBindings() {
		timeStamp.setDefaultCommand(new InstantCommand(
				() -> {
					timeStamp.update();
				},
				timeStamp));

		swerveSubsystem.setDefaultCommand(new DriveCommand(
				swerveSubsystem,
				() -> leftStick.getY(),
				() -> leftStick.getX(),
				() -> -rightStick.getX(),
				() -> leftStick.getRawButton(2)));

		clawSubsystem.setDefaultCommand(new ClawIntakeCommand(
				clawSubsystem,
				() -> controller.isPressed(LogiButton.A),
				() -> controller.isDown(LogiButton.LTrigger),
				() -> controller.isDown(LogiButton.RTrigger)));

		armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem,
				() -> MathUtil.applyDeadband(controller.getLeftStickY(), Constants.stickDeadband)
						* -Constants.armPivotOperatorCoefficient,
				() -> MathUtil.applyDeadband(controller.getRightStickY(), Constants.stickDeadband)
						* -Constants.armExtensionOperatorCoefficient,
				() -> controller.isPressed(LogiButton.B)));

		leftStick.isDownBind(StickButton.Bottom, new InstantCommand(() -> zeroGyro(), swerveSubsystem));
		controller.isPressedBind(LogiButton.X,
				new InstantCommand(() -> Led.setMode(Led.BlinkinLedMode.SOLID_VIOLET)));
		controller.isPressedBind(LogiButton.Y,
				new InstantCommand(() -> Led.setMode(Led.BlinkinLedMode.SOLID_ORANGE)));

		/*
		 * controller.isPressedBind(LogiButton.X, new InstantCommand(() ->
		 * Led.set(PatternID.OFF)));
		 * controller.isPressedBind(LogiButton.Y, new InstantCommand(() ->
		 * Led.set(PatternID.YELLOW)));
		 * controller.isPressedBind(LogiButton.B, new InstantCommand(() ->
		 * Led.set(PatternID.VIOLET)));
		 */
		// controller.isPressedBind(LogiButton.Start, new
		// PickUpCubeFromGround(armSubsystem, clawSubsystem));
		controller.isPressedBind(LogiButton.LBTrigger, new InstantCommand(() -> armSubsystem.setPivotDegrees(100)));
		// controller.isPressedBind(LogiButton.RBTrigger, new InstantCommand(() ->
		// armSubsystem.setPivotDegrees(0)));
		controller.isPressedBind(LogiButton.Back, new InstantCommand(() -> armSubsystem.setExtensionPosition(0)));
	}

	private void addAutonomousRoutines() {
		chooser.setDefaultOption("One Cone and Taxi (Stable)",
				new OneConeAndTaxiStable(armSubsystem, clawSubsystem, swerveSubsystem));
		chooser.setDefaultOption("One Cone and Taxi (Experimental)",
				new OneConeAndTaxiPP(armSubsystem, clawSubsystem, swerveSubsystem));

		chooser.setDefaultOption("One Cone and Charge Station",
				new OneConeAndChargeStation(armSubsystem, clawSubsystem, swerveSubsystem, pigeon2));

		chooser.addOption("None", null);

		SmartDashboard.putData("Auto Chooser", chooser);
	}

	public void zeroGyro() {
		pigeon2.setYaw(0);
		pigeon2.zeroGyroBiasNow();
		swerveSubsystem.zeroHeading();
	}

	public Pose2d getEstimatedPose() {
		return swerveSubsystem.getPose();
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}

	public void onTeleopInit() {
		Constants.isExtensionClosedLoop = loopTypeForExtension.getSelected() == "CL";
	}
}
