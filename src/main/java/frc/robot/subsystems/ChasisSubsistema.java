package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constantes.ConstantesChasis;
import frc.robot.Constantes.ConstantesRamsetController;

public class ChasisSubsistema extends SubsystemBase {
    
    // Motores en el lado Izquierdo del chasis
    private final static CANSparkMax motorIzqAdelante = new CANSparkMax(ConstantesChasis.kIdMotorIzqAdelante, MotorType.kBrushless);
    private final static CANSparkMax motorIzqAtras = new CANSparkMax(ConstantesChasis.kIdMotorIzqAtras, MotorType.kBrushless);
    private final static MotorControllerGroup motoresIzq = new MotorControllerGroup(motorIzqAdelante, motorIzqAtras);

    // Motores en el lado derecho del chasis
    private final static CANSparkMax motorDerAdelante = new CANSparkMax(ConstantesChasis.kIdMotorDerAdelante, MotorType.kBrushless);
    private final static CANSparkMax motorDerAtras = new CANSparkMax(ConstantesChasis.kIdMotorDerAtras, MotorType.kBrushless);
    private final static MotorControllerGroup motoresDer = new MotorControllerGroup(motorDerAdelante, motorDerAtras);
    
    // The robot's drive - Tren motriz del robot
    private final static DifferentialDrive chasis = new DifferentialDrive(motoresIzq, motoresDer);

    // Encoders del lado Izquierdo
    private final static RelativeEncoder encoderIzqAdel = motorIzqAdelante.getEncoder();
    private final static RelativeEncoder encoderIzqAtras = motorIzqAtras.getEncoder();

    // Encoders del lado Derecho
    private final static RelativeEncoder encoderDerAdel = motorDerAdelante.getEncoder();
    private final static RelativeEncoder encoderDerAtras = motorDerAtras.getEncoder();
    
    // Sensor Navx (gyro)
    private final static AHRS NavX = new AHRS(Port.kMXP);

    // Odometry class for tracking robot pose
    // Clase de odometría para rastrear la pose del robot
    private static DifferentialDriveOdometry m_odometry;

    double distIzqAdel, distIzqAtras, velIzqAdel, velIzqAtras;
    double distDerAdel, distDerAtras, velDerAdel, velDerAtras;
    double distProm;
    
    public ChasisSubsistema(){
        // Se restauraran los parametros por Default de los controladores
        // para asi evitar que alguna configuracion anterior afecte el codigo
        motorDerAdelante.restoreFactoryDefaults();
        motorDerAtras.restoreFactoryDefaults();
        motorIzqAdelante.restoreFactoryDefaults();
        motorIzqAtras.restoreFactoryDefaults();

        // Se asigna el modo de los controladores
        //      Brake: Detiene los motores inmediatamente al quitarle la energia
        //      Coast: DEteiene paulatinamente los motores
        motorDerAdelante.setIdleMode(IdleMode.kBrake);
        motorDerAtras.setIdleMode(IdleMode.kBrake);
        motorIzqAdelante.setIdleMode(IdleMode.kBrake);
        motorIzqAtras.setIdleMode(IdleMode.kBrake);

        // Se asigna el factor de conversion para obtener la distancia en metros
        encoderDerAdel.setPositionConversionFactor(ConstantesChasis.kFactorDeConversionAMetros);
        encoderDerAtras.setPositionConversionFactor(ConstantesChasis.kFactorDeConversionAMetros);
        encoderIzqAdel.setPositionConversionFactor(ConstantesChasis.kFactorDeConversionAMetros);
        encoderIzqAtras.setPositionConversionFactor(ConstantesChasis.kFactorDeConversionAMetros);

        // Se asigna el factor de conversion para obtener la distancia en metros/segundos
        encoderDerAdel.setVelocityConversionFactor(ConstantesChasis.kFactorDeConversionAMetros/60);
        encoderDerAtras.setVelocityConversionFactor(ConstantesChasis.kFactorDeConversionAMetros/60);
        encoderIzqAdel.setVelocityConversionFactor(ConstantesChasis.kFactorDeConversionAMetros/60);
        encoderIzqAtras.setVelocityConversionFactor(ConstantesChasis.kFactorDeConversionAMetros/60);
              
        // Necesitamos invertir un lado del tren motriz para que los voltajes
        // positivos den como resultado que ambos lados avancen.       
        // De esta forma no se invierten los controladores individualmente
        // Si alguien usa los controladores no se vera afectado
        motoresDer.setInverted(ConstantesChasis.kMotoresDerInvertidos);
        motoresIzq.setInverted(!motoresDer.getInverted());
        
        /*// De esta forma se invierten los motores individualmente y se aplican los cambios cada vez
        // que alguien usa los controladores
        motorDerAdelante.setInverted(ConstantesChasis.kMotoresDerInvertidos);
        motorDerAtras.setInverted(ConstantesChasis.kMotoresDerInvertidos);
        motorIzqAdelante.setInverted(!motorDerAdelante.getInverted());
        motorIzqAtras.setInverted(!motorDerAtras.getInverted());*/
        
        // Funcion para resetear los valores de los encoders del chasis
        resetEncoders();

        // Funcion para resetear el giroscopio
        resetGyro();

        // Crea la odometria con el giroscopio (Navx) y los encoders de cada lado
        // NECESITAMOS COLOCAR UN SIGNO NEGATIVO DEL ENCODER DERECHO PARA QUE AMBOS TENGAN EL MISMO SIGNO
        m_odometry =
            new DifferentialDriveOdometry(
                NavX.getRotation2d(), distIzqAdel, distDerAdel);
        
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Se lee la distancia y la velocidad de los encoders del chasis del lado Izquierdo
        distIzqAdel = encoderIzqAdel.getPosition();  
        distIzqAtras = encoderIzqAtras.getPosition();
        velIzqAdel = encoderIzqAdel.getVelocity();  
        velIzqAtras = encoderIzqAtras.getVelocity();


        // Se lee la distancia y la velocidad de los encoders del chasis del lado Derecho
        distDerAdel = -encoderDerAdel.getPosition();  
        distDerAtras = -encoderDerAtras.getPosition();
        velDerAdel = -encoderDerAdel.getVelocity();  
        velDerAtras = -encoderDerAtras.getVelocity();
        
        // Se calcula el promedio de los encoders del chasis
        distProm = (distDerAdel + distIzqAdel) / 2;
        
        SmartDashboard.putNumber("Mts Izq Adel", distIzqAdel);
        SmartDashboard.putNumber("Mts Izq Atras", distIzqAtras);
        SmartDashboard.putNumber("Mts Der Adel", distDerAdel);
        SmartDashboard.putNumber("Mts Der Atras", distDerAtras);

        SmartDashboard.putNumber("Vel Izq Adel", velIzqAdel);
        SmartDashboard.putNumber("Vel Izq Atras", velIzqAtras);
        SmartDashboard.putNumber("Vel Der Adel", velDerAdel);
        SmartDashboard.putNumber("Vel Der Atras", velDerAtras);

        SmartDashboard.putNumber("Mts Prom", distProm);

        SmartDashboard.putNumber("Navx gyro", NavX.getAngle());
        SmartDashboard.putNumber("Navx heading", getHeading());

        // Get the rotation of the robot from the gyro.
        var gyroAngle = NavX.getRotation2d();

        // Update the odometry in the periodic block
        m_odometry.update(gyroAngle, distIzqAdel, distDerAdel);
        
        System.out.println(getPose());

        
        
    }

    /**
    * Returns the currently-estimated pose of the robot.
    * return The pose.
    */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
    * Returns the current wheel speeds of the robot.
    * return The current wheel speeds. 
    */   
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(velIzqAdel, velDerAdel);
    }

    /**
    * Resets the odometry to the specified pose.
    * The pose to which to set the odometry.
    */
    public void resetOdometry(Pose2d pose){
        resetEncoders();
        m_odometry.resetPosition(NavX.getRotation2d(), encoderIzqAdel.getPosition(), -encoderDerAdel.getPosition(), pose);
    }

    /**
    * Drives the robot using arcade controls.
    *  fwd the commanded forward movement
    *  rot the commanded rotation
    */
    public void arcadeDrive(double fwd, double rot) {
        chasis.arcadeDrive(fwd, rot);
    }

    /**
    * Controls the left and right sides of the drive directly with voltages.
    * voltsIzq the commanded left output
    * voltsDer the commanded right output
    */
    public void tankDriveVolts(double voltsIzq, double voltsDer) {
        motoresIzq.setVoltage(-voltsIzq);
        motoresDer.setVoltage(-voltsDer);
        chasis.feed();
    }

    /**
    * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
    *  maxOutput the maximum output to which the drive will be constrained
    */
    public void setMaxOutput(double maxOutput) {
        chasis.setMaxOutput(maxOutput);
    }
    


    // Funcion para resetear los encoders del chasis
    /** Resets the drive encoders to currently read a position of 0. */
    public static void resetEncoders(){
        encoderDerAdel.setPosition(0);
        encoderDerAtras.setPosition(0);
        encoderIzqAdel.setPosition(0);
        encoderIzqAtras.setPosition(0);
    }


    public static void setMotoresVelocidadTanque(double velocidadIzq, double velocidadDer){
        chasis.tankDrive(velocidadIzq, velocidadDer);
    }

    public static void setMotoresVelocidadArcade(double velocidadX, double rotacion){
        chasis.arcadeDrive(velocidadX, rotacion);
    }

    

    /**
    * Returns the heading of the robot.
    * return the robot's heading in degrees, from -180 to 180
    */
    public double getHeading() {
        /*
        // no tiene inicio ni fin despues de 360 continua 361... Positivo Sentido manecillas del reloj
            SmartDashboard.putNumber("NavX.getAngle", NavX.getAngle()); 
        //lo mismo que getAngle pero con signo contrario. Positivo Sentido contrario a las manecillas del reloj
            SmartDashboard.putNumber("NavX.getRotation2d().getDegrees", NavX.getRotation2d().getDegrees()); 
        //de -180 a 180. Positivo Sentido manecillas del reloj
            SmartDashboard.putNumber("NavX.getYaw", NavX.getYaw()); 
        */

        NavX.getRotation2d().getDegrees(); //linea que indica WPILIB
        return NavX.getYaw(); //En navx para dar de -180 a 180 se usa esto
    }

    /**
    * Returns the turn rate of the robot.
    * return The turn rate of the robot, in degrees per second
    */
    public double getTurnRate() {
        return -NavX.getRate();
    }

    public static void resetGyro(){
        //NavX.zeroYaw();
        NavX.reset();
    }

    public static void resetOdometry(){
        m_odometry.resetPosition(NavX.getRotation2d(), encoderIzqAdel.getPosition(), -encoderDerAdel.getPosition(), new Pose2d());    
    }

    



    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory trayectoria, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(trayectoria.getInitialPose());
            }    
            }),
            new PPRamseteCommand(trayectoria, 
            this::getPose, 
            new RamseteController(ConstantesRamsetController.kRamseteB, ConstantesRamsetController.kRamseteZeta), 
            new SimpleMotorFeedforward(
                ConstantesRamsetController.ksVolts, 
                ConstantesRamsetController.kvVoltSecondsPerMeter,
                ConstantesRamsetController.kaVoltSecondsSquaredPerMeter), 
            ConstantesRamsetController.kDriveKinematics, 
            this::getWheelSpeeds, 
            new PIDController(0, 0, 0), 
            new PIDController(0, 0, 0), 
            this::tankDriveVolts,
            this
            )
        );
        /*return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialPose());
            }
            }),
            new PPRamseteCommand(
                traj, 
                this::getPose, // Pose supplier
                new RamseteController(ConstantesRamsetController.kRamseteB, ConstantesRamsetController.kRamseteZeta),
                new SimpleMotorFeedforward(
                    ConstantesRamsetController.ksVolts,
                    ConstantesRamsetController.kvVoltSecondsPerMeter,
                    ConstantesRamsetController.kaVoltSecondsSquaredPerMeter),
                ConstantesRamsetController.kDriveKinematics, // DifferentialDriveKinematics
                this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                this::outputVolts, // Voltage biconsumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );*/
    }


}
