// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constantes {
  
  public static class ConstantesControles {
    public static final int kPuertoControlDriver = 0;
    public static final int kEjeTankIzq = 1;
    public static final int kEjeTankDer = 5;
    public static final int kEjeArcAvance = 1;
    public static final int kEjeArcGiro = 4;


    public static final double kVelMax = 0.5;
  }

  public static class ConstantesChasis {
    public static final byte kIdMotorDerAdelante = 1;
    public static final byte kIdMotorDerAtras = 2;
    public static final byte kIdMotorIzqAdelante = 3;
    public static final byte kIdMotorIzqAtras = 4;

    public static final boolean kMotoresDerInvertidos = false;

    public static final double relacionDeTransmision = 12.84;
    public static double kFactorDeConversionAMetros = Units.inchesToMeters((1/(relacionDeTransmision*2*Math.PI*0.0762)*10));
    public static double kFactorDeConversionAMetrosDer = (kFactorDeConversionAMetros)*-1;

  }

  public static class ConstantesRamsetController{
    public static final double ksVolts = 0.099775;
    public static final double kvVoltSecondsPerMeter = 3.3227;
    public static final double kaVoltSecondsSquaredPerMeter = 0.21743;

    public static final double kPDriveVel = 0.67234;

    //Distancia entre llantas derechas e izquierdas 
    public static final double kTrackwidthMeters = Units.inchesToMeters(28);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

}
