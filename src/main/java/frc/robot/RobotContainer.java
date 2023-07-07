// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constantes.ConstantesControles;
import frc.robot.Constantes.ConstantesRamsetController;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDriveCmd;
import frc.robot.subsystems.ChasisSubsistema;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ChasisSubsistema main_chasisSubsistema =  new ChasisSubsistema();

  private final Joystick controlDriver = new Joystick(ConstantesControles.kPuertoControlDriver);

  // This will load the file "ExamplePath.path" and generate it with a max velocity of 3 m/s and a max acceleration of 1 m/s^2
  public static PathPlannerTrajectory examplePath = PathPlanner.loadPath("ExamplePath", new PathConstraints(3, 1));

  // This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
  // Or the path can be sampled at a given point in time for custom path following

  // Sample the state of the path at 1.2 seconds
  

  /*// This will load the file "Example Path Group.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Example Path Group", new PathConstraints(4, 3));

// This will load the file "Example Path Group.path" and generate it with different path constraints for each segment
List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup(
    "Example Path Group", 
    new PathConstraints(4, 3), 
    new PathConstraints(2, 2), 
    new PathConstraints(3, 3));*/
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    main_chasisSubsistema.resetEncoders();
    configureBindings();
    
    main_chasisSubsistema.setDefaultCommand(
      new TankDriveCmd(main_chasisSubsistema, () -> controlDriver.getRawAxis(ConstantesControles.kEjeTankIzq) * ConstantesControles.kVelMax, () -> controlDriver.getRawAxis(ConstantesControles.kEjeTankDer) * ConstantesControles.kVelMax)
    );
  }

  private void configureBindings() {

    new JoystickButton(controlDriver, 1).whileFalse(
      new TankDriveCmd(main_chasisSubsistema, () -> controlDriver.getRawAxis(ConstantesControles.kEjeTankIzq) * ConstantesControles.kVelMax, () -> controlDriver.getRawAxis(ConstantesControles.kEjeTankDer) * ConstantesControles.kVelMax)
      );
    new JoystickButton(controlDriver, 2).whileFalse(
      new ArcadeDriveCmd(main_chasisSubsistema, () -> controlDriver.getRawAxis(ConstantesControles.kEjeArcAvance) * ConstantesControles.kVelMax, () -> controlDriver.getRawAxis(ConstantesControles.kEjeArcGiro) * ConstantesControles.kVelMax)
      );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              ConstantesRamsetController.ksVolts, 
              ConstantesRamsetController.kvVoltSecondsPerMeter, 
              ConstantesRamsetController.kaVoltSecondsSquaredPerMeter),
            ConstantesRamsetController.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                ConstantesRamsetController.kMaxSpeedMetersPerSecond,
                ConstantesRamsetController.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(ConstantesRamsetController.kDriveKinematics)
            //Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)), 
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.5, 0),  new Translation2d(0, 0)),
            //List.of(new Translation2d(0,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)), 
            config);



    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            main_chasisSubsistema::getPose,
            new RamseteController(ConstantesRamsetController.kRamseteB, ConstantesRamsetController.kRamseteZeta),
            new SimpleMotorFeedforward(
                ConstantesRamsetController.ksVolts,
                ConstantesRamsetController.kvVoltSecondsPerMeter,
                ConstantesRamsetController.kaVoltSecondsSquaredPerMeter),
            ConstantesRamsetController.kDriveKinematics,
            main_chasisSubsistema::getWheelSpeeds,
            new PIDController(ConstantesRamsetController.kPDriveVel, 0, 0),
            new PIDController(ConstantesRamsetController.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            main_chasisSubsistema::tankDriveVolts,
            main_chasisSubsistema);

    // Reset odometry to the starting pose of the trajectory.
    main_chasisSubsistema.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> main_chasisSubsistema.tankDriveVolts(0, 0));
    

    /*// Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              ConstantesRamsetController.ksVolts, 
              ConstantesRamsetController.kvVoltSecondsPerMeter, 
              ConstantesRamsetController.kaVoltSecondsSquaredPerMeter),
            ConstantesRamsetController.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                ConstantesRamsetController.kMaxSpeedMetersPerSecond,
                ConstantesRamsetController.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(ConstantesRamsetController.kDriveKinematics)
            //Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)), 
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.5, 0.5),  new Translation2d(1, -0.3)),
            //List.of(new Translation2d(0,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)), 
            config);



    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            main_chasisSubsistema::getPose,
            new RamseteController(ConstantesRamsetController.kRamseteB, ConstantesRamsetController.kRamseteZeta),
            new SimpleMotorFeedforward(
                ConstantesRamsetController.ksVolts,
                ConstantesRamsetController.kvVoltSecondsPerMeter,
                ConstantesRamsetController.kaVoltSecondsSquaredPerMeter),
            ConstantesRamsetController.kDriveKinematics,
            main_chasisSubsistema::getWheelSpeeds,
            new PIDController(ConstantesRamsetController.kPDriveVel, 0, 0),
            new PIDController(ConstantesRamsetController.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            main_chasisSubsistema::tankDriveVolts,
            main_chasisSubsistema);

    // Reset odometry to the starting pose of the trajectory.
    main_chasisSubsistema.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> main_chasisSubsistema.tankDriveVolts(0, 0));*/

    //EJEMPLO return Autos.exampleAuto(m_exampleSubsystem);


    return main_chasisSubsistema.followTrajectoryCommand(examplePath, false);
  }
}
