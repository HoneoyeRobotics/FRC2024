// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private final Shooter m_Subsystem;
  private final double speed;

  /** Creates a new IndependentShooter. */
  public RunShooter(Shooter subsystem, double speed) {
    m_Subsystem = subsystem;
    addRequirements(subsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_Subsystem.noteCheck() && speed < 0) {
      m_Subsystem.runtopmotor(0);
      m_Subsystem.runbottommotor(0);

    } else {
      m_Subsystem.runtopmotor(speed);
      m_Subsystem.runbottommotor(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Subsystem.runtopmotor(0);
    m_Subsystem.runbottommotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
