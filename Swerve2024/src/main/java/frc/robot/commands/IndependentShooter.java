// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class IndependentShooter extends Command {
  private final Shooter m_Subsystem;
  private final DoubleSupplier m_RStick;
  private final DoubleSupplier m_LStick;
  private final BooleanSupplier m_RBumper;
  double topSpeed = 0;
  double bottomSpeed = 0;
  double topSpeedIn = 0;
  double bottomSpeedIn = 0;
  double topSpeedOut = 0;
  double bottomSpeedOut = 0;
  double leftStick = 0;
  double rightStick = 0;
  boolean timerBool = false;
  int i = 0;

  /** Creates a new IndependentShooter. */
  public IndependentShooter(Shooter subsystem, DoubleSupplier rStick, DoubleSupplier lStick, BooleanSupplier rBumper) {
    m_Subsystem = subsystem;
    m_RStick = rStick;
    m_LStick = lStick;
    m_RBumper = rBumper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerBool = false;
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topSpeed = 0;
    bottomSpeed = 0;
    topSpeed = m_RStick.getAsDouble() * -1;
    bottomSpeed = m_LStick.getAsDouble()* -1;
    // leftStick = m_LStick.getAsDouble();
    // if ((leftStick < 0.08 ) && (leftStick > -0.08)) leftStick = 0;
    // rightStick = m_RStick.getAsDouble();
    // if ((rightStick < 0.08 ) && (rightStick > -0.08)) rightStick = 0;

    // if (leftStick > 0)
    // bottomSpeed = Constants.ShooterSpeeds.bottomSpeedIn;
    // else if (leftStick < 0)
    // bottomSpeed = Constants.ShooterSpeeds.bottomSpeedOut;

    // if (rightStick > 0)
    // topSpeed = Constants.ShooterSpeeds.topSpeedIn;
    // else if (rightStick < 0)
    // topSpeed = Constants.ShooterSpeeds.topSpeedOut;

    if (m_RBumper.getAsBoolean() == true) {
      rightStick = m_RStick.getAsDouble();
      if ((rightStick < 0.08) && (rightStick > -0.08))
        rightStick = 0;
      if (rightStick > 0)
        topSpeed = Constants.ShooterConstants.topSpeedIn;
      else if (rightStick < 0)
        topSpeed = Constants.ShooterConstants.topSpeedOut;
      bottomSpeed = topSpeed;
    }

    if (m_Subsystem.noteCheck() == true)
      timerBool = true;

    if (timerBool) {
      if (topSpeed > 0 || bottomSpeed > 0) {
        topSpeed = 0;
        bottomSpeed = 0;
      }
      i++;
      if (i > 100 || topSpeed < 0) {
        timerBool = false;
        i = 0;
      }
    }

    m_Subsystem.runtopmotor(topSpeed);
    m_Subsystem.runbottommotor(bottomSpeed);
    SmartDashboard.putBoolean("sensor", m_Subsystem.note);
    SmartDashboard.putNumber("Bottomspeed", m_LStick.getAsDouble());
    SmartDashboard.putNumber("Topspeed", m_RStick.getAsDouble());
    SmartDashboard.putNumber("BSI", bottomSpeedIn);
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
