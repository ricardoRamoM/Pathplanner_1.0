// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constantes.ConstantesControles;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDriveCmd;
import frc.robot.subsystems.ChasisSubsistema;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ChasisSubsistema main_chasisSubsistema =  new ChasisSubsistema();

  private final Joystick controlDriver = new Joystick(ConstantesControles.kPuertoControlDriver);
  

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
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
