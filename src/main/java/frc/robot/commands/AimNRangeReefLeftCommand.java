package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// Positions Robot at the Nearest Valid Target
public class AimNRangeReefLeftCommand extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_driveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller stuff (woah so many so scary)
  PIDController m_aimController = new PIDController(VisionConstants.kP_aim, VisionConstants.kI_aim, VisionConstants.kD_aim);
  PIDController m_rangeController = new PIDController(VisionConstants.kP_range, VisionConstants.kI_range, VisionConstants.kD_range);
  PIDController m_strafeController = new PIDController(VisionConstants.kP_strafe, VisionConstants.kI_strafe, VisionConstants.kD_strafe);
    
  // All the Valid IDs available for positioning
  int[] validIDs = {6, 7, 8, 9, 10, 11};

  // Bot Pose Target Space Relative (TX, TY, TZ, Pitch, Yaw, Roll)
  private double[] botPoseTargetSpace = new double[6];

  // Lil boolean for checking for "Tag In View" 
  boolean tiv;

  // Constructor
  public AimNRangeReefLeftCommand(DriveSubsystem driveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command
  public void initialize() {
    
    // Adds condition that filters out undesired IDs
    LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.k_limelightName, validIDs);

    // Checks for TIV
    tiv = LimelightHelpers.getTV(VisionConstants.k_limelightName);

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    // Update the pose from NetworkTables (Limelight Readings)
    botPoseTargetSpace = NetworkTableInstance.getDefault().getTable(VisionConstants.k_limelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    // If tags are in view, drive right over!
    if (tiv) m_driveSubsystem.drive(limelight_range_PID(), limelight_strafe_PID(), limelight_aim_PID(), false, true);

    // Otherwise we tell it to quit
    else tiv = false;
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached 
  public boolean isFinished() {
    return (
      // Range (Distance to Tag)
      botPoseTargetSpace[2] < VisionConstants.k_rangeReefLeftThresholdMax
      && botPoseTargetSpace[2]  > VisionConstants.k_rangeReefLeftThresholdMin
      
      // Aim (Angle)
      && botPoseTargetSpace[4]  < VisionConstants.k_aimReefLeftThresholdMax
      && botPoseTargetSpace[4]  > VisionConstants.k_aimReefLeftThresholdMin

      // Strafe (Left Right Positioning)
      && botPoseTargetSpace[0]  < VisionConstants.k_strafeReefLeftThresholdMax
      && botPoseTargetSpace[0]  > VisionConstants.k_strafeReefLeftThresholdMin)

      // Other quit conditions
      || !tiv || timer.get() > 3;
  }

  // Advanced PID-assisted ranging control with Limelight's TZ value from target-relative data
  private double limelight_range_PID() {
    m_rangeController.enableContinuousInput(-30, 30); // TODO: Check these numbers?
    
    // Calculates response based on difference in distance from tag to robot
    double targetingForwardSpeed = m_rangeController.calculate(botPoseTargetSpace[2] - VisionConstants.k_rangeReefLeftTarget);

    // Value scale up to robot max speed and invert
    targetingForwardSpeed *= -1.0 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Hooray
    return targetingForwardSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's TX value from target-relative data
  private double limelight_strafe_PID() {
    m_strafeController.enableContinuousInput(-30, 30);
    
    // Calculates response based on difference in horizontal distance from tag to robot
    double targetingStrafeSpeed = m_strafeController.calculate(botPoseTargetSpace[0] - VisionConstants.k_strafeReefLeftTarget);

    // Value scale up to robot max speed
    targetingStrafeSpeed *= 1.0 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Hooray
    return targetingStrafeSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's Yaw value from target-relative data
  private double limelight_aim_PID() {
    m_aimController.enableContinuousInput(-40, 40);
    
    // Calculates response based on difference in angle from tag to robot
    double targetingAngularVelocity = m_aimController.calculate(botPoseTargetSpace[4] - VisionConstants.k_aimReefLeftTarget);

    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= -0.1 * DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
