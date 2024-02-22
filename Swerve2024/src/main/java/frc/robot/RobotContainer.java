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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arms;
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

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.*;

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

        public void teleopInit() {
                // m_arms.resetLowerElbowEncoder();

                // m_arms.resetUpperElbowEncoder();
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
                // m_shooter.setDefaultCommand(new IndependentShooter(m_shooter,
                //                 () -> m_codriverController.getRightY(),
                //                 () -> m_codriverController.getLeftY() / 1.4,
                //                 () -> m_codriverController.b().getAsBoolean()));
                // Configure default commands
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
                SmartDashboard.putData("Reset Shoulder Encoder", new ResetLowerElbowEncoder(m_arms));
                SmartDashboard.putData("Reset Elbow Encoder", new ResetUpperElbowEncoder(m_arms));

                SmartDashboard.putData("Pickup Shoulder", new SetShoulder(m_arms, 20));
                SmartDashboard.putData("Pickup Elbow", new SetElbow(m_arms, -7));



                SmartDashboard.putData("Amp Shoulder", new SetShoulder(m_arms, 27));
                SmartDashboard.putData("Amp Elbow", new SetElbow(m_arms, -32));


                SmartDashboard.putData("Speaker Shoulder", new SetShoulder(m_arms, -23));
                SmartDashboard.putData("Speaker Elbow", new SetElbow(m_arms, 10));

                SmartDashboard.putData("Reset Gyro", new ResetGyro(m_robotDrive));

                SmartDashboard.putData("StabTheNote", new FireTheThingThatStabsTheNote(m_shooter));

                SequentialCommandGroup AmpToHome = new SequentialCommandGroup(
                       // new MoveArmsToPosition(m_arms, 27,-32, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 20, -28, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 12, -18, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 6, -12, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, -5, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, 0, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Amp To Home",  AmpToHome);

        

                 SequentialCommandGroup HomeToAmp = new SequentialCommandGroup(
                      //  new MoveArmsToPosition(m_arms, 0,0, RobotPrefs.getEndTolerance()),
                     //   new MoveArmsToPosition(m_arms, 0,-4, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0,-9, RobotPrefs.getMiddleTolerance()),
                        //new MoveArmsToPosition(m_arms, 6,-12, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 12,-18, RobotPrefs.getMiddleTolerance()),
                       new MoveArmsToPosition(m_arms, 20,-28, RobotPrefs.getMiddleTolerance()),
                           new MoveArmsToPosition(m_arms, 27,-32, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Home To Amp",  HomeToAmp);



                SequentialCommandGroup PickUpToHome = new SequentialCommandGroup(
                       // new MoveArmsToPosition(m_arms, 19,-7, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 15, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 12, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 6, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, 0, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("PickUp To Home",  PickUpToHome);

  SequentialCommandGroup HomeToPickUp = new SequentialCommandGroup(
                       // new MoveArmsToPosition(m_arms, 0,0, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 0, -5, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 6, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 12, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 18, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 19, -7, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Home To PickUp",  HomeToPickUp);


                  SequentialCommandGroup FeederToHome = new SequentialCommandGroup(
                       // new MoveArmsToPosition(m_arms, 21,-26, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 14, -22, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 8, -15, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 2, -6, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, -3, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, 0, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Feeder To Home",  FeederToHome);

                 SequentialCommandGroup HomeToFeeder = new SequentialCommandGroup(
                     //   new MoveArmsToPosition(m_arms, 0,0, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 0,-3, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 2,-6, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 8,-15, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 14,-22, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 21,-26, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Home To Feeder", HomeToFeeder);


                SequentialCommandGroup SpeakerToHome = new SequentialCommandGroup(
                        //new MoveArmsToPosition(m_arms, 12, -23, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 11, -18, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 6, -12, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 3, -7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, -4, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, 0, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Speaker To Home",  SpeakerToHome);

                SequentialCommandGroup HomeToSpeaker = new SequentialCommandGroup(
                      //  new MoveArmsToPosition(m_arms, 0,0, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 0,-4, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 3,-7, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 6,-12, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 11,-18, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 12,-23, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Home To Speaker",  HomeToSpeaker);

                
                
                SequentialCommandGroup driveToAmp = new SequentialCommandGroup(
                      //  new MoveArmsToPosition(m_arms, 6,-14, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 12, -20, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 18, -26, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 27, -32, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Drive to Amp",  driveToAmp);

                 SequentialCommandGroup AmpToPickUp = new SequentialCommandGroup(
                       // new MoveArmsToPosition(m_arms, 27,-32, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 24, -26, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 22, -18, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 22, -10, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Amp To Pick Up",  AmpToPickUp);

                 SequentialCommandGroup AmpToDrive = new SequentialCommandGroup(
                     //   new MoveArmsToPosition(m_arms, 27,-32, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 24, -26, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 18, -18, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 10, -10, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 0, -10, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Amp To Drive",  AmpToDrive);

                 SequentialCommandGroup PickUpToAmp = new SequentialCommandGroup(
                      //  new MoveArmsToPosition(m_arms, 22, -10, RobotPrefs.getEndTolerance()),
                        new MoveArmsToPosition(m_arms, 10, -10, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 12, -15, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 14, -20, RobotPrefs.getMiddleTolerance()),
                        new MoveArmsToPosition(m_arms, 37, -32, RobotPrefs.getEndTolerance())
                );

                SmartDashboard.putData("Pick Up To Amp",  PickUpToAmp);

                SmartDashboard.putData("Toggle stab pid", new ToggleStabPID(m_shooter));

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
                buttonBoard.button(11).onTrue(new MoveShoulder(m_arms, 1));
                buttonBoard.button(10).onTrue(new MoveShoulder(m_arms, -1));
                buttonBoard.button(3).onTrue(new MoveElbow(m_arms, 1));
                buttonBoard.button(9).onTrue(new MoveElbow(m_arms, -1));
                buttonBoard.button(5).onTrue(new ToggleArmPID(m_arms));
                //shoot top
                buttonBoard.button(7).onTrue(new ShootSequence(m_shooter, RobotPrefs.getSpeakerWait(), RobotPrefs.getSpeakerTimeout(), 1));
                //shoot amp
                buttonBoard.button(6).onTrue(new ShootSequence(m_shooter, RobotPrefs.getAmpWait(), RobotPrefs.getAmpTimeout(), RobotPrefs.getShooterAmpSpeed()));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(Units.inchesToMeters(60), 0)
                                        
                                        
                                        ),
                                // End 2 meters straight ahead of where we started, facing forward
                                new Pose2d(Units.inchesToMeters(72), Units.inchesToMeters(-48), new Rotation2d(180)),
                                config);

                Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(2, 0, new Rotation2d(90)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(
                                        new Translation2d(0.5,0.5)
                                        
                                        
                                        ),
                                // End 2 meters straight ahead of where we started, facing forward
                                new Pose2d(0, 0, new Rotation2d(0)),
                                config);


                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);


                                  SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                                exampleTrajectory2,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);


                // Reset odometry to the starting pose of the trajectory.
                m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return swerveControllerCommand
               // .andThen(swerveControllerCommand2)
                .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false, true));
        }
 public Command getAutonomousCommand3() {
    return new PathPlannerAuto("Auto 2");
  }

}
