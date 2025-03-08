// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */

public class RunElevator extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Elevator elevator;
  private final double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunElevator(Elevator elevator,double speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



    // Called every time the scheduler runs while the command is scheduled.

  @Override
    public void execute() {
        elevator.setElevatorSpeed(speed);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }


}