// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arms;

public class MoveArmsToPosition extends Command {
  /** Creates a new MoveArmsToPosition. */
  private Arms m_arms;
  private double shoulderSetpoint;
  private double elbowSetpoint;
  private double tolerance;
  public MoveArmsToPosition(Arms arms, double shoulderSetpoint, double elbowSetpoint, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arms);
    m_arms = arms;
    this.shoulderSetpoint = shoulderSetpoint;
    this.elbowSetpoint = elbowSetpoint;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arms.setTolerance(tolerance);
      m_arms.moveelbow(elbowSetpoint, true);
      m_arms.moveshoulder(shoulderSetpoint, true);


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
    return m_arms.iselbowAtPosition() && m_arms.isshoulderAtPosition();
  }
}
