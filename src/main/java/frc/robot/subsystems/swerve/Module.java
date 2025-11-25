package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {
    
    private SparkMax steer;
    private SparkMax drive;

    private double steerSet = 0;
    private double driveSet = 0;

    private PIDController steerPID = new PIDController(4.5, 0, 0.07);
    private PIDController drivePID = new PIDController(0, 0, 0.0);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 1/6.75);

    public Module(int sid, int did){
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
        steer = new SparkMax(sid, MotorType.kBrushless);
        drive = new SparkMax(did, MotorType.kBrushless);

    }

    public void periodic(){
        double pos = steer.getAbsoluteEncoder().getPosition() - Math.PI;
        double vel = drive.getEncoder().getVelocity();
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/position", pos);
        Logger.recordOutput("subsystems/drive"+steer.getDeviceId()+"/positionSet", steerSet);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/speed", vel);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/speedSet", -driveSet);
        
        double goSteer = steerPID.calculate(pos, steerSet);
        double goDrive = drivePID.calculate(vel, -driveSet) + driveFF.calculate(-driveSet);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/PIDOut", goSteer);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/PIDOut", goDrive);
        
        steer.setVoltage(goSteer);
        drive.setVoltage(goDrive);
        
    }
    public void setSteerPoint(double point){
        steerSet = point;
    }
    public void setDrivePoint(double point){
        driveSet = point;
    }
    public void setState(SwerveModuleState state){
        setSteerPoint(state.angle.getRadians());
        setDrivePoint(state.speedMetersPerSecond/Units.inchesToMeters(2));
    }
}
