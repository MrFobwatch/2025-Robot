// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.OperatorConstants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.ClosedLoopSlot;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Elevator extends SubsystemBase {

  private final SparkMax leaderMotor;
  private SparkMax followerMotor;
  private final SparkMaxConfig leaderConfig;
  private final SparkMaxConfig followerConfig;
  private final SparkMaxConfig globalConfig;
  private final SparkClosedLoopController pidController;

  private static final double kP = 0.1;  // Proportional gain - tune as needed
  private static final double kI = 0.0;  // Integral gain - usually 0 at first
  private static final double kD = 0.0;  // Derivative gain - tune as needed
  
  
  public Elevator(){

    leaderMotor = new SparkMax(OperatorConstants.ELEVATOR_MOTOR1_ID, MotorType.kBrushless);
    followerMotor = new SparkMax(OperatorConstants.ELEVATOR_MOTOR2_ID, MotorType.kBrushless);

    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();
    globalConfig = new SparkMaxConfig();

    pidController = leaderMotor.getClosedLoopController();

    globalConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .encoder.positionConversionFactor(OperatorConstants.inchesPerMotorRotation);

    leaderConfig
        .apply(globalConfig);

    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD);
    
  
    followerConfig
        .apply(globalConfig)
        .follow(leaderMotor,true);

    
    
    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



  }

 public double getHeightInches() {
	return leaderMotor.getEncoder().getPosition();
  }


  public void setElevatorSpeed(double speed){
    leaderMotor.set(speed);
  }

  public void stopElevator(){
    leaderMotor.set(0);
  }


  public void setElevatorPosition(double position){
    pidController.setReference(position,ControlType.kPosition);
  }

  //function to reset the elevator position
  public void resetElevatorPosition() {
	leaderMotor.getEncoder().setPosition(0);
  }

  public boolean isAtHeight(double targetHeight, double tolerance) {
	double currentHeight = getHeightInches();
	return Math.abs(currentHeight - targetHeight) <= tolerance;
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
