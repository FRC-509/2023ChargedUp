package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.Odometry;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.server.PathPlannerServer;

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

  public final GenericHID operatorController = new GenericHID(2);
  private final JoystickButton leftTrigger = new JoystickButton(leftStick, 1);
  private final JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
  private final JoystickButton leftStickButtonTwo = new JoystickButton(leftStick, 2);

  public RobotContainer() {
    // Initialize and configure the gyroscope.
    this.pigeon2.configFactoryDefault();
    // Zero the gyroscope rotation.
    this.zeroGyro();

    // Instantiate the odometer.
    this.odometry = new Odometry(pigeon2);
    // Instantiate the drivetrain.
    this.swerveSubsystem = new Swerve(pigeon2);

    // Configure button/stick bindings.
    configureButtonBindings();
  }

  public void configureButtonBindings() {
    // Set the default command of the drive train subsystem to DriveCommand.
    this.swerveSubsystem.setDefaultCommand(new DriveCommand(
        this.swerveSubsystem,
        () -> this.leftStick.getY(),
        () -> this.leftStick.getX(),
        () -> this.rightStick.getX(),
        () -> this.leftStick.getRawButton(2)));

    // When button two on the left stick is pressed, zero the gyroscope.
    // The swerve subsystem is added as a requirement, since although the Pigeon
    // does not live in it, it most certainly depends on it.
    this.leftStickButtonTwo.whileTrue(
        new InstantCommand(() -> zeroGyro(),
            this.swerveSubsystem));

    /*
     * // The slider on the right stick controls the intake motor speed. Intake with
     * the right stick's trigger, outtake with the left stick's trigger.
     * rightTrigger.whileTrue(new IntakeCommand(() ->
     * this.rightStick.getThrottle()));
     * leftTrigger.whileTrue(new IntakeCommand(() ->
     * -this.rightStick.getThrottle()));
     */
  }

  public void zeroGyro() {
    this.pigeon2.setYaw(0);
    this.pigeon2.zeroGyroBiasNow();
  }

  /*
   * public void initializeDriveTrain() {
   * this.swerveSubsystem.resetIntegratedToAbsolute();
   * PathPlannerServer.startServer(5811);
   * }
   */
  public Command getAutonomousCommand() {
    return new TrajectoryBuilderWrapper("New Path").getPathFollowingCommand(this.swerveSubsystem);
  }
}
