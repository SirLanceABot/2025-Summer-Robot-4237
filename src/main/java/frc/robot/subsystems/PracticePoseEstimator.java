package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.motors.TalonFXLance;
import frc.robot.sensors.Camera;

/**
 * This is an example of what a subsystem should look like.
 */
public class PracticePoseEstimator extends SubsystemLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }
    

    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here


    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final Camera[] cameraArray;
    private final Pigeon2 gyro;
    private final Drivetrain drivetrain;

    private final SwerveDrivePoseEstimator poseEstimator;

    private Matrix<N3, N1> visionStdDevs;
    private Matrix<N3, N1> stateStdDevs;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new PracticePoseEstimator. 
     */
    public PracticePoseEstimator(Drivetrain drivetrain, Camera[] cameraArray)
    {
        super("Practice Pose Estimator");
        System.out.println("  Constructor Started:  " + fullClassName);

        this.cameraArray = cameraArray;
        this.drivetrain = drivetrain;
        this.gyro = drivetrain.getPigeon2();

        configStdDevs();

        if (drivetrain != null && gyro != null)
        {
            poseEstimator = new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(), 
                gyro.getRotation2d(), 
                drivetrain.getState().ModulePositions,
                drivetrain.getState().Pose,
                stateStdDevs,
                visionStdDevs);

            drivetrain.setVisionMeasurementStdDevs(visionStdDevs);
            drivetrain.setStateStdDevs(stateStdDevs); 
        }
        else
        {
            poseEstimator = null;
        }

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    private void configStdDevs()
    {
        visionStdDevs.set(0, 0, 0.15);
        visionStdDevs.set(1, 0, 0.15);
        visionStdDevs.set(2, 0, 0.2);

        stateStdDevs.set(0, 0, 0.1); 
        stateStdDevs.set(1, 0, 0.1); 
        stateStdDevs.set(2, 0, 0.05); 
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here


    // Use a method reference instead of this method
    // public Command stopCommand()
    // {
    //     return run( () -> stop() );
    // }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        for(Camera camera : cameraArray)
        {
            if(camera != null && drivetrain != null)
            {
                LimelightHelpers.SetRobotOrientation(camera.getCameraName(), drivetrain.getState().Pose.getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
            }

            if(camera.getTagCount() > 0)
            {
                Pose2d visionPose = camera.getPose();
                int tagID = (int) camera.getTagId();
            }
        }
    }

    @Override
    public String toString()
    {
        return "";
    }
}
