// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  Drivetrain dt;
  XboxController driverController;
  boolean fieldRelative=true;
  public ArcadeDrive(Drivetrain dt, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt=dt;
    this.driverController=driverController;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driverController.getStartButton())
      fieldRelative=!fieldRelative;
    dt.drive(driverController.getLeftX()*MAX_SPEED
      , driverController.getLeftY()*MAX_SPEED
      , driverController.getRightX()*MAX_ANGULAR_SPEED
      , fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
