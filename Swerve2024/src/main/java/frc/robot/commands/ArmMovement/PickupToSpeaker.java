// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmMovement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.RobotPrefs;
import frc.robot.commands.MoveArmsToPosition;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Arms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupToSpeaker extends SequentialCommandGroup {
  /** Creates a new AmpToHome. */
  private Arms m_arms;

  public PickupToSpeaker(Arms m_arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arms = m_arms;
    addCommands(

        new ToggleArmPosition(m_arms, ArmPosition.ToSpeaker),
        // new MoveArmsToPosition(m_arms, 19,-7, RobotPrefs.getEndTolerance()),
        new MoveArmsToPosition(m_arms, 15, .129, RobotPrefs.getMiddleTolerance()),
        // new MoveArmsToPosition(m_arms, 15,-12, RobotPrefs.getMiddleTolerance()),
        new MoveArmsToPosition(m_arms, ArmPositions.SpeakerS , ArmPositions.SpeakerE, RobotPrefs.getEndTolerance()).withTimeout(1.5),
        new ToggleArmPosition(m_arms, ArmPosition.Speaker));
  }
}
