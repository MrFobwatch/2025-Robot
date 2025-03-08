// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeightCommand extends Command {
  /** Creates a new SetElevatorHeightCommand. */
    private final Elevator m_elevator;
    private final double targetHeight;
  public SetElevatorHeightCommand(Elevator elevator,double targetHeight) {
    m_elevator = elevator;
    this.targetHeight = targetHeight;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_arm.isElevatorMovementSafe()) {
          m_elevator.setElevatorPosition(targetHeight);
          } else {
          // Hold current position if movement isn't safe
          m_elevator.setElevatorPosition(m_elevator.getHeightInches());
          }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command completes when elevator reaches target height with tolerance
    return m_elevator.isAtHeight(targetHeight, 0.5);  // Using 0.5 inch tolerance
  }
}
