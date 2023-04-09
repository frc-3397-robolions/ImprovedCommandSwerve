// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Drivetrain drivetrain = new Drivetrain();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
  private final CommandJoystick operatorJoystick = new CommandJoystick(OPERATOR_CONTROLLER_PORT);
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();
    SmartDashboard.putData(autoChooser);
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,driverController));
    // Configure the trigger bindings
    configureBindings();
    configureAutoCommands();

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));
      

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  private void configureAutoCommands(){
    AUTO_EVENT_MAP.put("shoot", new PrintCommand("*shoots*"));
    AUTO_EVENT_MAP.put("pickup", new PrintCommand("*picks up*"));
    AUTO_EVENT_MAP.put("drop", new PrintCommand("*drops*"));

    List<PathPlannerTrajectory> leftAuto = PathPlanner.loadPathGroup("2CubeAutoLeft", AUTO_MAX_VEL, AUTO_MAX_ACCEL);
    List<PathPlannerTrajectory> midAuto = PathPlanner.loadPathGroup("2CubeAutoMiddle", AUTO_MAX_VEL, AUTO_MAX_ACCEL);
    List<PathPlannerTrajectory> rightAuto = PathPlanner.loadPathGroup("2CubeAutoRight", AUTO_MAX_VEL, AUTO_MAX_ACCEL);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getPose, // Pose2d supplier
    drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    drivetrain::driveFromChassisSpeeds, // Module states consumer used to output to the drive subsystem
    AUTO_EVENT_MAP,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
  );
    Command TwoCubeAutoLeft = autoBuilder.fullAuto(leftAuto);
    autoChooser.setDefaultOption("TwoCubeAutoLeft", TwoCubeAutoLeft);

    Command TwoCubeAutoMid = autoBuilder.fullAuto(midAuto);
    autoChooser.addOption("TwoCubeAutoMid", TwoCubeAutoMid);

    Command TwoCubeAutoRight = autoBuilder.fullAuto(rightAuto);
    autoChooser.addOption("TwoCubeAutoRight", TwoCubeAutoRight);
  
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
