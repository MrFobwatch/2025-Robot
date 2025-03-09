// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
  /** Creates a new RotatingArm. */
  // Initialize the arm motor controller
  private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
  private TalonFXConfiguration armTalonFXConfig = new TalonFXConfiguration();

  private final MotionMagicExpoTorqueCurrentFOC armMotionMagicRequest;

  private double targetAngle = 0.0; // Target angle for the arm in degrees
  
  
  public Arm() {
    // Configure the arm motor to use motion magic control with FOC Postion
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    // Initialize MotionMagic FOC control request
    armMotionMagicRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    armMotor.getConfigurator().apply(armTalonFXConfig);

    //Need to add code to set the amount of roatations based on the gear ratio

    
  }

  /**
   * Sets the target Angle for the arm.
   * @param Angle The target Angle in rotations
   */
  public void setTargetAngle(double Angle) {
    // Clamp the Angle to the allowed range
    targetAngle = MathUtil.clamp(
        Angle,
        ArmConstants.ARM_MIN_ANGLE,
        ArmConstants.ARM_MAX_ANGLE);

      armMotionMagicRequest.withPosition(targetAngle);
  }
  

  @Override
  public void periodic() {
    // armMotionMagicRequest.withPosition(targetAngle);
  }

  public void setArmSpeed(double speed) {
    // Replace with actual logic to set the arm speed
    armMotor.set(speed);
  }

  public void stopArm() {
    // Replace with actual logic to stop the arm
    armMotor.set(0);
  }

  public double getArmAngle() {
    // Replace with actual logic to get the arm angle
    return armMotor.getPosition().getValueAsDouble() * ArmConstants.ARM_GEAR_RATIO; // Convert to degrees
  }

  public boolean isElevatorMovementSafe() {
    if (this.getArmAngle() < 45) { // Replace with actual angle check
        return true;
    } else {
        return false;
    }
  }
}
