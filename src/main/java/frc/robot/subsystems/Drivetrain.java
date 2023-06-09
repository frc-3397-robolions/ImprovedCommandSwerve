// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PhotonCameraWrapper;

import static frc.robot.Constants.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private final Translation2d m_frontLeftLocation = new Translation2d(1, 1.5);
  private final Translation2d m_frontRightLocation = new Translation2d(1, -1.5);
  private final Translation2d m_backLeftLocation = new Translation2d(-1, 1.5);
  private final Translation2d m_backRightLocation = new Translation2d(-1, -1.5);

  private final SwerveModule m_frontLeft = new SwerveModule("fl");
  private final SwerveModule m_frontRight = new SwerveModule("fr");
  private final SwerveModule m_backLeft = new SwerveModule("bl");
  private final SwerveModule m_backRight = new SwerveModule("br");

  private final AHRS m_gyro = new AHRS(Port.kMXP);
  
  public PhotonCameraWrapper pcw;

  private Pose2d m_pose = new Pose2d(1.95,4.43,new Rotation2d(0));

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          m_pose
          );

  public Drivetrain() {
    m_gyro.reset();
    pcw = new PhotonCameraWrapper();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pose=m_poseEstimator.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });
    Optional<EstimatedRobotPose> result =
      pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

if (result.isPresent()) {
  EstimatedRobotPose camPose = result.get();
  m_poseEstimator.addVisionMeasurement(
          camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
}
Logger.getInstance().recordOutput("Pose", m_pose);
Logger.getInstance().recordOutput("States", new SwerveModuleState[]{
  m_frontLeft.getState(),
  m_frontRight.getState(),
  m_backLeft.getState(),
  m_backRight.getState()
});
  }
public Pose2d getPose(){
  return m_pose;
}
public void resetPose(Pose2d pose){
  m_pose = pose;
}
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    Logger.getInstance().recordOutput("Desired States", new SwerveModuleState[]{
      swerveModuleStates[0],
      swerveModuleStates[1],
      swerveModuleStates[2],
      swerveModuleStates[3]});
    // Logger.getInstance().recordOutput("Desired Poses", new double[]{
    //   swerveModuleStates[0].angle.getRadians(),
    //   swerveModuleStates[0].speedMetersPerSecond,
    //   swerveModuleStates[1].angle.getRadians(),
    //   swerveModuleStates[1].speedMetersPerSecond,
    //   swerveModuleStates[2].angle.getRadians(),
    //   swerveModuleStates[2].speedMetersPerSecond,
    //   swerveModuleStates[3].angle.getRadians(),
    //   swerveModuleStates[3].speedMetersPerSecond,
    // });
  }
  public void driveFromChassisSpeeds(ChassisSpeeds speeds){
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
  public void setModuleStates(SwerveModuleState[] swerveModuleStates){
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
}
