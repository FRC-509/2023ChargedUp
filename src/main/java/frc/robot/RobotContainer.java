package frc.robot;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OdometryCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.Odometry;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  public final Pigeon2 pigeon2 = new Pigeon2(0);
  public final Joystick leftStick = new Joystick(1);
  public final Joystick rightStick = new Joystick(0);

  public final Swerve swerveSubsystem;
  public final Odometry odometry;
  public final Intake intakeSubsystem;
  public final Arm armSubsystem;
  public final Claw clawSubsystem;

  public final GenericHID operatorController = new GenericHID(2);
  private final JoystickButton leftTrigger = new JoystickButton(leftStick, 1);
  private final JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
  private final JoystickButton leftStickButtonTwo = new JoystickButton(leftStick, 2);
  private final JoystickButton leftStickButtonThree = new JoystickButton(leftStick, 3);
  private final JoystickButton operatorButtonOne = new JoystickButton(operatorController, 1);
  private final JoystickButton operatorButtonTwo = new JoystickButton(operatorController, 2);

  private boolean clawOpen = true;

  public RobotContainer() {
    // Initialize and configure the gyroscope.
    this.pigeon2.configFactoryDefault();
    // Zero the gyroscope rotation.
    this.zeroGyro();

    // Instantiate the odometer.
    this.odometry = new Odometry(pigeon2);
    // Instantiate the drivetrain.
    this.swerveSubsystem = new Swerve(pigeon2);
    // Instantiate the intake.
    this.intakeSubsystem = new Intake();
    // Instantiate the arm.
    this.armSubsystem = new Arm();
    // Instantiate the claw.
    this.clawSubsystem = new Claw();
    // Configure button/stick bindings.
    this.configureButtonBindings();
  }

  public void configureButtonBindings() {
    // Set the default command of the drive train subsystem to DriveCommand.
    this.swerveSubsystem.setDefaultCommand(new DriveCommand(
        this.swerveSubsystem,
        () -> this.leftStick.getY(),
        () -> this.leftStick.getX(),
        () -> this.rightStick.getX(),
        () -> this.leftStick.getRawButton(2)));

    this.odometry.setDefaultCommand(new OdometryCommand(this.odometry, this.swerveSubsystem.swerveOdometry));

    // When button two on the left stick is pressed, zero the gyroscope.
    // The swerve subsystem is added as a requirement, since although the Pigeon
    // does not live in it, it most certainly depends on it.
    this.leftStickButtonTwo.whileTrue(
        new InstantCommand(() -> zeroGyro(),
            this.swerveSubsystem));

    // The slider on the right stick controls the intake motor speed. Intake with
    // the right stick's trigger, outtake with the left stick's trigger.
    this.rightTrigger
        .whileTrue(new IntakeCommand(this.intakeSubsystem, () -> (this.rightStick.getThrottle() + 1.0d) / 2.0d));
    this.leftTrigger
        .whileTrue(new IntakeCommand(this.intakeSubsystem, () -> -(this.rightStick.getThrottle() + 1.0d) / 2.0d));

    // The A button on the operator's Logitech controller, or button three on the
    // driver's left stick, is used for toggling the claw's state between open and
    // closed.
    this.operatorButtonOne.onTrue(
        new InstantCommand(() -> {
          this.clawOpen = !this.clawOpen;
          this.clawSubsystem.setState(clawOpen);
        },
            this.armSubsystem))
        .or(leftStickButtonThree);

    this.armSubsystem
        .setDefaultCommand(new ArmCommand(armSubsystem, () -> (this.operatorController.getRawAxis(0) * 360.0d),
            () -> this.operatorController.getRawAxis(1)));
  }

  public void zeroGyro() {
    this.pigeon2.setYaw(0);
    this.pigeon2.zeroGyroBiasNow();
  }

  public Command getAutonomousCommand() {
    return new TrajectoryBuilderWrapper("New Path").getPathFollowingCommand(this.swerveSubsystem);
  }
}
