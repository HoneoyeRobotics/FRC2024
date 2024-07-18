// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BottomDrives;

public class DriveBottom extends Command {
  private BottomDrives m_drivetrain;
  private DoubleSupplier m_leftTriggerSupplier;
  private DoubleSupplier m_rightTriggerSupplier;
  private DoubleSupplier m_leftStickXSupplier;

  private double oldspeed = 0.0;
  private double oldspeedz = 0.0;
  private double xspeed;
  private double zrotation;
  private double tmpSpeed;

  public DriveBottom(BottomDrives drivetrain, DoubleSupplier leftTriggerSupplier, DoubleSupplier rightTriggerSupplier,
      DoubleSupplier leftStickXSupplier) {
    m_leftTriggerSupplier = leftTriggerSupplier;
    m_rightTriggerSupplier = rightTriggerSupplier;
    m_leftStickXSupplier = leftStickXSupplier;

    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  private double deadband = 0.1;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    oldspeed = xspeed;
    oldspeedz = zrotation;
    xspeed = m_rightTriggerSupplier.getAsDouble() - m_leftTriggerSupplier.getAsDouble();
    zrotation = (m_leftStickXSupplier.getAsDouble());

    // // xSpeed = xSpeed * xSpeed * (xSpeed > 0 ? 1 : -1);
    // tmpSpeed = xspeed * xspeed;
    // if (xspeed < 0)
    //   xspeed = tmpSpeed * -1.0;
    // else
    //   xspeed = tmpSpeed;

    // zrotation = zrotation * zrotation * (zrotation > 0 ? 1 : -1);
    // if (m_drivetrain.getFast() == false) {
    // xspeed = xspeed / 1.75;
    // zrotation = zrotation / 2;
    // }
    if ((zrotation < deadband) && (zrotation > (deadband * -1)))
      zrotation = 0;
    if ((xspeed < deadband) && (xspeed > (deadband * -1)))
      xspeed = 0;

  
    m_drivetrain.drive(xspeed, zrotation * -1);
  }

  

  public static final double ramprate = .02;
  public static final double rampratez = .035;

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
