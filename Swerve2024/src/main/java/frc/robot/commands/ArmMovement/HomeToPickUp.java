// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmMovement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPrefs;
import frc.robot.commands.MoveArmsToPosition;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Arms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeToPickUp extends SequentialCommandGroup {
  /** Creates a new AmpToHome. */
  private Arms m_arms;

  public HomeToPickUp(Arms m_arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arms = m_arms;
    addCommands(
      
        new ToggleArmPosition(m_arms, ArmPosition.ToPickup),
    new MoveArmsToPosition(m_arms, 0, -5, RobotPrefs.getMiddleTolerance()),
    new MoveArmsToPosition(m_arms, 6, -7, RobotPrefs.getMiddleTolerance()),
    new MoveArmsToPosition(m_arms, 12, -7, RobotPrefs.getMiddleTolerance()),
    new MoveArmsToPosition(m_arms, 18, -7, RobotPrefs.getMiddleTolerance()),
    new MoveArmsToPosition(m_arms, 19, -7, RobotPrefs.getEndTolerance()).withTimeout(1.5),
    new ToggleArmPosition(m_arms, ArmPosition.Pickup)
    );
  }
}
