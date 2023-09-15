// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

import java.util.HashMap;

public class SwerveModule extends SubsystemBase{

  private SparkMaxPIDController m_turningPIDController;
  private SparkMaxPIDController m_drivePIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  public String moduleID;

  private static final double kModuleMaxAngularVelocity = MAX_ANGULAR_SPEED;
  private static final double kModuleMaxAngularAcceleration = 2*Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelativeEncoder;
  private final CANCoder m_turningAbsoluteEncoder;


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID CAN ID for the drive motor.
   * @param turningMotorID CAN ID for the turning motor.
   */
  public SwerveModule(String moduleID) {
    this.moduleID=moduleID;
    switch(moduleID){
      case("fl"):
      m_driveMotor = new CANSparkMax(FRONT_LEFT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(FRONT_LEFT_ANGLE_ID,MotorType.kBrushless);
      m_turningAbsoluteEncoder = new CANCoder(23);
      m_turningAbsoluteEncoder.configMagnetOffset(34.5-71);
      break;
      case("fr"):
      m_driveMotor = new CANSparkMax(FRONT_RIGHT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(FRONT_RIGHT_ANGLE_ID,MotorType.kBrushless);
      m_turningAbsoluteEncoder = new CANCoder(21);
      m_turningAbsoluteEncoder.configMagnetOffset(192.65-25);
      break;
      case("bl"):
      m_driveMotor = new CANSparkMax(BACK_LEFT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(BACK_LEFT_ANGLE_ID,MotorType.kBrushless);
      m_turningAbsoluteEncoder = new CANCoder(22);
      m_turningAbsoluteEncoder.configMagnetOffset(254.4+43.5-192);
      break;
      case("br"):
      m_driveMotor = new CANSparkMax(BACK_RIGHT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(BACK_RIGHT_ANGLE_ID,MotorType.kBrushless);
      m_turningAbsoluteEncoder = new CANCoder(24);
      m_turningAbsoluteEncoder.configMagnetOffset(159.2+41);
      break;
      default:
      throw new Error("Module must be initialized with an ID");
      
    }
    //IDK it might fix something
    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    //Keep voltage reasonable
    m_driveMotor.enableVoltageCompensation(12);
    m_driveMotor.setSmartCurrentLimit(60);

    //create drive encoder as a relative encoder
    //using Meters
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS/21);
   
    //Makes a relative encoder and sets it to where the absolute encoder is
    //using radians
    m_turningRelativeEncoder=m_turningMotor.getEncoder();
    m_turningRelativeEncoder.setPositionConversionFactor(2*Math.PI/21);
    m_turningRelativeEncoder.setPosition(m_turningAbsoluteEncoder.getAbsolutePosition()*Math.PI/180);
    
    //makes a sparkmaxpidcontroller for angles
    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningRelativeEncoder);

    m_turningPIDController.setP(0.005);
    m_turningPIDController.setI(0);
    m_turningPIDController.setD(0);
    m_turningPIDController.setIZone(0);
    m_turningPIDController.setFF(0);
    m_turningPIDController.setOutputRange(1, -1);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setFeedbackDevice(m_driveEncoder);
    m_drivePIDController.setP(0.1);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    SmartDashboard.putNumber(moduleID, m_turningAbsoluteEncoder.getAbsolutePosition());
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_turningAbsoluteEncoder.getAbsolutePosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), Rotation2d.fromDegrees(m_turningAbsoluteEncoder.getAbsolutePosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(m_turningAbsoluteEncoder.getAbsolutePosition()));

    // Calculate the drive output from the drive PID controller.
    m_drivePIDController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

    // Calculate the turning motor output from the turning PID controller.
    m_turningPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }
}
