package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
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
  public static final Joystick leftStick = new Joystick(1);
  public static final Joystick rightStick = new Joystick(0);

  private final GenericHID genericHid = new GenericHID(2);
  private JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
  // private JoystickButton leftStickButtonTwo = new JoystickButton(leftStick, 2);

  public RobotContainer() {
    // Configure the trigger bindings
    swerveSubsystem.setDefaultCommand(new DriveCommand(
        swerveSubsystem,
        () -> leftStick.getY(),
        () -> leftStick.getX(),
        () -> rightStick.getX(),
        () -> leftStick.getRawButton(2)));
  }

  public void initializeDriveTrain() {
    swerveSubsystem.resetIntegratedToAbsolute();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
