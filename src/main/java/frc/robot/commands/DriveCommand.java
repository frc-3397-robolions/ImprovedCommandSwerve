// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  double xSpeed, ySpeed, oSpeed;
  Drivetrain sub;
  public DriveCommand(Drivetrain sub, double xSpeed, double yspeed, double oSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xSpeed=xSpeed;
    this.ySpeed=yspeed;
    this.oSpeed=oSpeed;
    this.sub=sub;
    addRequirements(sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sub.drive(xSpeed*AUTO_MAX_VEL,ySpeed*AUTO_MAX_VEL,oSpeed*AUTO_MAX_VEL,true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
