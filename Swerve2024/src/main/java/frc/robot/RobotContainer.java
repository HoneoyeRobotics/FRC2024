// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.*;
import frc.robot.commands.ArmMovement.ToggleArmPosition;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        private static final Sendable PickUpToHome = null;
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final Shooter m_shooter = new Shooter();
        public final Arms m_arms = new Arms();
        public final Climber m_climber = new Climber();
        private SendableChooser<Command> auto = new SendableChooser<>();

        public void teleopInit() {
                // m_arms.resetLowerElbowEncoder();

                // m_arms.resetUpperElbowEncoder();
                m_arms.FixArmInTeleop();
                m_arms.resetallpositions();
        }

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        CommandXboxController m_codriverController = new CommandXboxController(OIConstants.kcoDriverControllerPort);

        private CommandJoystick buttonBoard = new CommandJoystick(OIConstants.kButtonBoardControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                NamedCommands.registerCommand("ArmAmp", new ArmAmp(m_arms));
                NamedCommands.registerCommand("ShootAmp", new ShootAmp(m_shooter));
                NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker(m_shooter));
                NamedCommands.registerCommand("ArmSpeaker", new ArmSpeaker(m_arms));
                NamedCommands.registerCommand("ArmHome", new ArmHome(m_arms));
                NamedCommands.registerCommand("ArmPickup", new ArmPickup(m_arms));
                NamedCommands.registerCommand("RunPickup", new RunShooter(m_shooter, -0.5));

                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                // -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                // OIConstants.kDriveDeadband)
                                                                -MathUtil.applyDeadband(m_driverController
                                                                                .getRightTriggerAxis()
                                                                                - m_driverController
                                                                                                .getLeftTriggerAxis(),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true,
                                                                m_driverController.leftBumper().getAsBoolean()),
                                                m_robotDrive));
                SmartDashboard.putData("Reset Shoulder Encoder",
                                new ResetLowerElbowEncoder(m_arms).ignoringDisable(true));
                SmartDashboard.putData("Reset Elbow Encoder", new ResetUpperElbowEncoder(m_arms).ignoringDisable(true));

                // SmartDashboard.putData("Deploy Climber", new DeployClimber(m_climber));
                // SmartDashboard.putData("Retract Climber", new RetractClimber(m_climber,
                // true).withTimeout(3));

                SmartDashboard.putData("Reset Climber Enc", new ResetClimberEncoders(m_climber));
                // SmartDashboard.putData("Retract and hold", new RetractClimber(m_climber,
                // true));

                SmartDashboard.putData("Reset Gyro", new ResetGyro(m_robotDrive).ignoringDisable(true));
                // auto.addOption("auto 1", new PathPlannerAuto("Auto 1"));
                auto.setDefaultOption("Center back, center, feeder", new PathPlannerAuto("Auto 2"));
                
                auto.setDefaultOption("Center back, center (lame)", new PathPlannerAuto("Auto 2 lame"));
                auto.setDefaultOption("AUTO STAY", new PathPlannerAuto("AUTO STAY"));
                
                auto.setDefaultOption("Center back, center, amp", new PathPlannerAuto("Auto 2-Up"));
                auto.addOption("Feeder, center, far 2nd Amp", new PathPlannerAuto("Auto 3"));
                auto.addOption("Amp back", new PathPlannerAuto("Auto 4"));
                auto.addOption("Feeder back", new PathPlannerAuto("Auto 5"));
                // auto.addOption("auto 6", new PathPlannerAuto("Auto 6"));
                auto.addOption("Center back Amp", new PathPlannerAuto("Auto 7"));
                auto.addOption("Center far 2nd amp", new PathPlannerAuto("Auto 8"));
                auto.addOption("Feeder far feeder", new PathPlannerAuto("Auto 9"));
                auto.addOption("Feeder 2nd far", new PathPlannerAuto("Auto 10"));
                auto.addOption("Feeder far middle", new PathPlannerAuto("Auto 11"));
                auto.addOption("Shoot than stop-AMP", new PathPlannerAuto("Shoot than stop-AMP"));
                auto.addOption("Shoot than stop-Feeder", new PathPlannerAuto("Shoot than stop-Feeder"));
                auto.addOption("KRT", new PathPlannerAuto("KRT"));
                // auto.addOption("Speaker-amp-mid", new PathPlannerAuto("Speaker-amp-mid"));
                SmartDashboard.putData("Auto Mode", auto);
                // SmartDashboard.putData(m_climber);

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {

                m_driverController.rightBumper()
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                m_driverController.b().whileTrue(new RunShooter(m_shooter, -0.5));
                m_driverController.y().whileTrue(new RunShooter(m_shooter, 0.5));
                buttonBoard.axisLessThan(0, -0.2).onTrue(new MoveShoulder(m_arms, 1));
                buttonBoard.axisGreaterThan(0, 0.2).onTrue(new MoveShoulder(m_arms, -1));
                buttonBoard.axisLessThan(1, -0.2).onTrue(new MoveElbow(m_arms, -1));
                buttonBoard.axisGreaterThan(1, 0.2).onTrue(new MoveElbow(m_arms, 1));
                m_driverController.back().onTrue(new ResetGyro(m_robotDrive));
                // buttonBoard.button(8).onTrue(new ToggleArmPID(m_arms));
                // shoot top
                buttonBoard.button(7).onTrue(new ShootSpeaker(m_shooter));
                // shoot amp
                buttonBoard.button(8).onTrue(new ShootAmp(m_shooter));

                buttonBoard.button(11).onTrue(new ArmHome(m_arms));
                buttonBoard.button(3).onTrue(new ArmPickup(m_arms));
                buttonBoard.button(2).onTrue(new ArmAmp(m_arms));
                buttonBoard.button(6).onTrue(new ArmSpeaker(m_arms));
                buttonBoard.button(10).onTrue(new ArmFeeder(m_arms));
                buttonBoard.button(9).onTrue(new ToggleArmPosition(m_arms,
                ArmPosition.Home));

                buttonBoard.button(5).onTrue(new RetractClimber(m_climber, true).withTimeout(3));
                buttonBoard.button(4).onTrue(new DoTheClimb(m_climber, m_arms));
                m_driverController.axisGreaterThan(5, 0.2).whileTrue(new RunRightClimber(m_climber, 0.1));
                m_driverController.axisLessThan(5, -0.2).whileTrue(new RunRightClimber(m_climber, -0.1));
                m_driverController.axisGreaterThan(4, 0.2).whileTrue(new RunLeftClimber(m_climber, 0.1));
                m_driverController.axisLessThan(4, -0.2).whileTrue(new RunLeftClimber(m_climber, -0.1));
        }

        public Command getAutonomousCommand() {
                return auto.getSelected();
        }


}
