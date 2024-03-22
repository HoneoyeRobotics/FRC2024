// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class DeployClimber extends Command {
  private Climber climber;
  private double LeftSetpoint;
  private double RightSetpoint;
    /** Creates a new Climb. */
  public DeployClimber(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(climber);
    LeftSetpoint = 28;
    RightSetpoint = -27;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
    climber.holdClimber = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setLeftClimber(climber.getLeftClimberPosition() >= LeftSetpoint ? 0 : 0.25);
    climber.setRightClimber(climber.getRightClimberPosition() <= RightSetpoint ? 0 : -0.25);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftClimber(0);
    climber.setRightClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getRightClimberPosition() <= RightSetpoint && climber.getLeftClimberPosition() >= LeftSetpoint;
  }
}
