package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Swerve;

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

  public final Swerve swerveSubsystem = new Swerve();
  public final Joystick leftStick = new Joystick(1);
  public final Joystick rightStick = new Joystick(0);

  public final GenericHID operatorController = new GenericHID(2);
  private final JoystickButton leftTrigger = new JoystickButton(leftStick, 1);
  private final JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
  private final JoystickButton leftStickButtonTwo = new JoystickButton(leftStick, 2);

  public RobotContainer() {
    // Configure the trigger bindings
    swerveSubsystem.setDefaultCommand(new DriveCommand(
        swerveSubsystem,
        () -> leftStick.getY(),
        () -> leftStick.getX(),
        () -> rightStick.getX(),
        () -> leftStick.getRawButton(2)));

    leftStickButtonTwo.whileTrue(
        new InstantCommand(() -> swerveSubsystem.zeroGyro(),
            swerveSubsystem));

    /*
     * // The slider on the right stick controls the intake motor speed. Intake with
     * the right stick's trigger, outtake with the left stick's trigger.
     * rightTrigger.whileTrue(new IntakeCommand(() -> rightStick.getThrottle()));
     * leftTrigger.whileTrue(new IntakeCommand(() -> -rightStick.getThrottle()));
     */
  }

  public void initializeDriveTrain() {
    swerveSubsystem.resetIntegratedToAbsolute();
    PathPlannerServer.startServer(5811);
  }

  public Command getAutonomousCommand() {
    return new TrajectoryBuilderWrapper("New Path").getPathFollowingCommand(swerveSubsystem);
  }
}
