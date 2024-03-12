// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RetractClimber extends Command {
  private Climber climber;
  private boolean Hold;
  private PIDController LeftClimbPID;
  private PIDController RightClimbPID;
  /** Creates a new Climb. */
  public RetractClimber(Climber climber, boolean Hold) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(climber);
    this.Hold = Hold;
    LeftClimbPID = new PIDController(0.2, 0, 0);
    RightClimbPID = new PIDController(0.2, 0, 0);
    LeftClimbPID.setTolerance(2);
    RightClimbPID.setTolerance(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LeftClimbPID.setSetpoint(-2);
    RightClimbPID.setSetpoint(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setLeftClimber(LeftClimbPID.calculate(climber.getLeftClimberPosition()));
    climber.setRightClimber(RightClimbPID.calculate(climber.getRightClimberPosition()));
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
    if (Hold == true)
    return false;
    return LeftClimbPID.atSetpoint() && RightClimbPID.atSetpoint();
  }
}
