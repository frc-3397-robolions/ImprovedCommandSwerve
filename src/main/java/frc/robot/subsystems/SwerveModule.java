// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import java.util.HashMap;

public class SwerveModule extends SubsystemBase{
  
  private static HashMap<Integer,Double> encoderOffsets = new HashMap<>(4);
  private static final double kModuleMaxAngularVelocity = MAX_ANGULAR_SPEED;
  private static final double kModuleMaxAngularAcceleration =
      2*Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.005,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.123, 0.134);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID CAN ID for the drive motor.
   * @param turningMotorID CAN ID for the turning motor.
   */
  public SwerveModule(String moduleID) {

    switch(moduleID){
      case("fl"):
      m_driveMotor = new CANSparkMax(FRONT_LEFT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(FRONT_LEFT_ANGLE_ID,MotorType.kBrushless);
      m_turningEncoder = new CANCoder(23);
      m_turningEncoder.configMagnetOffset(34.5);
      break;
      case("fr"):
      m_driveMotor = new CANSparkMax(FRONT_RIGHT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(FRONT_RIGHT_ANGLE_ID,MotorType.kBrushless);
      m_turningEncoder = new CANCoder(21);
      m_turningEncoder.configMagnetOffset(192.65);
      break;
      case("bl"):
      m_driveMotor = new CANSparkMax(BACK_LEFT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(BACK_LEFT_ANGLE_ID,MotorType.kBrushless);
      m_turningEncoder = new CANCoder(22);
      m_turningEncoder.configMagnetOffset(254.4);
      break;
      case("br"):
      m_driveMotor = new CANSparkMax(BACK_RIGHT_SPEED_ID,MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(BACK_RIGHT_ANGLE_ID,MotorType.kBrushless);
      m_turningEncoder = new CANCoder(24);
      m_turningEncoder.configMagnetOffset(159.2);
      break;
      default:
      throw new Error("Module must be initialized with an ID");
    }
    m_driveEncoder = m_driveMotor.getEncoder();
    

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);

    

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity()/60, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPosition(), state.angle.getRadians());

    //final double turnFeedforward =
        //m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput);
  }
}
