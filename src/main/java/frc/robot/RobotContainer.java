// Imports stuff (again!)

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Controls;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LEDColorChangeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {

  // Robot's Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // DEPRECATED: Calibrates Gyro
    m_robotDrive.calibrateGyro();

    // Configure Default Commands
    m_robotDrive.setDefaultCommand(
      
        // The left stick on the controller controls robot translation.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(

            // Joystick input to tele-op control
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true), m_robotDrive));

    SmartDashboard.putData("AutoMode", m_chooser);

    // Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(4, 3.2, Rotation2d.fromDegrees(180));

    // Create the velocity and acceleration (translational and angular) constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command limelightDrive = AutoBuilder.pathfindToPose(
      targetPose,
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );

    // Named Command Configuration
    NamedCommands.registerCommand("Change LED Color", new LEDColorChangeCommand(m_LEDSubsystem));
    // NamedCommands.registerCommand("Limelight Auto", limelightDrive);

    // Autos
    m_chooser.addOption("Curvy yay", m_robotDrive.getAuto("Curvy yay"));
    m_chooser.addOption("Move and Spin", m_robotDrive.getAuto("Move and Spin"));
    m_chooser.addOption("Limelight Drive", m_robotDrive.getAuto("Limelight Drive"));

  }

  // Use this method to define your button to command mappings. Buttons can be
  // created by instantiating a edu.wpi.first.wpilibj.GenericHID or one of its
  // subclasses edu.wpi.first.wpilibj.Joystick or XboxController, and then
  // passing it to a JoystickButton.

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Controls.setXValue)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            // Sets wheels in an X position to prevent movement
            // A button driver
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();   
  } 
}