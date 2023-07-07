package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constantes.ConstantesChasis;

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
    private final AHRS NavX = new AHRS(Port.kMXP);

    // Odometry class for tracking robot pose
    // Clase de odometría para rastrear la pose del robot
    private final DifferentialDriveOdometry m_odometry;

    
    
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
        motoresDer.setInverted(ConstantesChasis.kMotoresDerInvertidos);
        motoresIzq.setInverted(!motoresDer.getInverted());
        
        /*// De esta forma se invierten los motores individualmente y se aplican los cambios cada vez
        // que alguien usa los controladores
        motorDerAdelante.setInverted(ConstantesChasis.kMotoresDerInvertidos);
        motorDerAtras.setInverted(ConstantesChasis.kMotoresDerInvertidos);
        motorIzqAdelante.setInverted(!motorDerAdelante.getInverted());
        motorIzqAtras.setInverted(!motorDerAtras.getInverted());*/
        
        resetEncoders();
        resetGyro();

        m_odometry = new DifferentialDriveOdometry(NavX.getRotation2d(), encoderIzqAdel.getPosition(), -encoderDerAdel.getPosition());
        
        m_odometry.resetPosition(NavX.getRotation2d(), encoderIzqAdel.getPosition(), -encoderDerAdel.getPosition(), new Pose2d());
    }

    /**
    * Returns the current wheel speeds of the robot. 
    */   
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(encoderIzqAdel.getVelocity(), encoderDerAdel.getVelocity());
    }

    public double getHeading() {
        return NavX.getRotation2d().getDegrees();
    }

    /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetGyro(){
        //NavX.zeroYaw();
        NavX.reset();
    }

    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param voltsIzq the commanded left output
    * @param voltsDer the commanded right output
    */
    public void tankDriveVolts(double voltsIzq, double voltsDer) {
        motoresIzq.setVoltage(voltsIzq);
        motoresDer.setVoltage(voltsDer);
        chasis.feed();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double distIzqAdel = encoderIzqAdel.getPosition();  
        double distIzqAtras = encoderIzqAtras.getPosition();
        double distDerAdel = -encoderDerAdel.getPosition();  
        double distDerAtras = -encoderDerAtras.getPosition();
        
        double distProm = (distDerAdel + distIzqAdel) / 2;
        
        SmartDashboard.putNumber("Mts Izq Adel", distIzqAdel);
        SmartDashboard.putNumber("Mts Izq Atras", distIzqAtras);
        SmartDashboard.putNumber("Mts Der Adel", distDerAdel);
        SmartDashboard.putNumber("Mts Der Atras", distDerAtras);

        SmartDashboard.putNumber("Vel Izq Adel", encoderIzqAdel.getVelocity() );
        SmartDashboard.putNumber("Vel Izq Atras", encoderIzqAtras.getVelocity());
        SmartDashboard.putNumber("Vel Der Adel", -encoderDerAdel.getVelocity() );
        SmartDashboard.putNumber("Vel Der Atras", -encoderDerAtras.getVelocity());

        SmartDashboard.putNumber("Mts Prom", distProm);
        
        SmartDashboard.putNumber("Navx gyro", NavX.getAngle());
        SmartDashboard.putNumber("Navx heading", getHeading());

        System.out.println(getPose());

         // Get the rotation of the robot from the gyro.
         var gyroAngle = NavX.getRotation2d();
         //System.out.println(gyroAngle);

        // Update the pose
        Pose2d m_pose = m_odometry.update(gyroAngle,
        encoderIzqAdel.getPosition(),
        encoderDerAdel.getPosition());
        
        //System.out.println(m_pose);

    }

    

    public static void setMotoresVelocidadTanque(double velocidadIzq, double velocidadDer){
        chasis.tankDrive(velocidadIzq, velocidadDer);
    }

    public static void setMotoresVelocidadArcade(double velocidadX, double rotacion){
        chasis.arcadeDrive(velocidadX, rotacion);
    }

    public static void resetEncoders(){
        encoderDerAdel.setPosition(0);
        encoderDerAtras.setPosition(0);
        encoderIzqAdel.setPosition(0);
        encoderIzqAtras.setPosition(0);
    }

    

    
}
