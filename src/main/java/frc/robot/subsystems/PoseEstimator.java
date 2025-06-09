package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.sensors.Camera;

/**
 * This is an example of what a subsystem should look like.
 */
public class PoseEstimator extends SubsystemLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public enum branchSide
    {
        kLeft,
        kRight;
    }

    private final Pigeon2 gyro;
    private final Drivetrain drivetrain;
    private final Camera[] cameraArray;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final NetworkTable ASTable;
    private final double fieldXDimension = 17.5482504;
    private final double fieldYDimension = 8.0519016;
    private final double[] defaultPosition = {0.0, 0.0, 0.0};
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Kalman filter, experiment later
    private Matrix<N3, N1> visionStdDevs;
    private Matrix<N3, N1> stateStdDevs;

    private int totalTagCount = 0;

    // Inputs
    private Rotation2d gyroRotation;
    private SwerveModulePosition[] swerveModulePositions;

    // Outputs
    private Pose2d estimatedPose = new Pose2d();
    private StructPublisher<Pose2d> poseEstimatorEntry;

    private boolean isRightBranch = false;
    private int primaryFace = 1;
    private int primaryReefTag = 0;

    // private final double[][] blueLeftBranchLocationArray = 
    // {{5.447446, 4.1859}, //S1 side
    //  {5.06044, 3.1693908}, //S2 side
    //  {3.888206, 3.1693908}, //S3 side
    //  {3.5312, 4.1859}, //S4 side
    //  {3.9381227, 4.9124092}, //S5 side
    //  {5.0405233, 4.9124092}}; //S6 side

    // private final double[][] blueRightBranchLocationArray = 
    // {{5.447446, 3.8659}, //S1 side
    //  {5.0405233, 3.1393908}, //S2 side
    //  {3.9381227, 3.1393908}, //S3 side
    //  {3.5312, 3.8659}, //S4 side
    //  {3.888206, 4.8824092}, //S5 side
    //  {5.09044, 4.8824092}}; //S6 side

    //  private final double[][] redLeftBranchLocationArray = 
    // {{12.100906, 3.8659}, //S1 side
    //  {12.457658, 4.8827472}, //S2 side
    //  {13.660146, 4.8824092}, //S3 side
    //  {14.016898, 3.8659}, //S4 side
    //  {13.6102433, 3.1393908}, //S5 side
    //  {12.5075747, 3.1393908}}; //S6 side

    // private final double[][] redRightBranchLocationArray = 
    // {{12.100906, 4.1859}, //S1 side
    //  {12.5075747, 4.9124092}, //S2 side
    //  {13.6102293, 4.9124092}, //S3 side
    //  {14.016898, 4.1859}, //S4 side
    //  {13.660146, 3.1693908}, //S5 side
    //  {12.457658, 3.1693908}}; //S6 side

     private final HashMap<Integer, Pose2d> leftBranchMap = new HashMap<Integer, Pose2d>();
     private final HashMap<Integer, Pose2d> rightBranchMap = new HashMap<Integer, Pose2d>();
     
    //  left.put(6, new Pose2d( new Translation2d(13.6102433, 3.1393908), new Rotation2d(Math.toRadians(30))));
 
    private final List<Pose2d> aprilTagLocations = new ArrayList<Pose2d>(12){{
        add(new Pose2d(new Translation2d(13.474446, 3.306318), new Rotation2d(Math.toRadians(300.0)))); // Red S5
        add(new Pose2d(new Translation2d(13.890498, 4.0259), new Rotation2d(Math.toRadians(0.0)))); // Red S4
        add(new Pose2d(new Translation2d(13.474446, 4.745482), new Rotation2d(Math.toRadians(60.0)))); // Red S3
        add(new Pose2d(new Translation2d(12.643358, 4.745482), new Rotation2d(Math.toRadians(120.0)))); // Red S2
        add(new Pose2d(new Translation2d(12.227306, 4.0259), new Rotation2d(Math.toRadians(180.0)))); // Red S1
        add(new Pose2d(new Translation2d(12.643358, 3.306318), new Rotation2d(Math.toRadians(240.0)))); // Red S6

        add(new Pose2d(new Translation2d(4.073906, 3.306318), new Rotation2d(Math.toRadians(240.0)))); // Blue S3
        add(new Pose2d(new Translation2d(3.6576, 4.0259), new Rotation2d(Math.toRadians(180.0)))); // Blue S4
        add(new Pose2d(new Translation2d(4.073906, 4.745482), new Rotation2d(Math.toRadians(120.0)))); // Blue S5
        add(new Pose2d(new Translation2d(4.90474, 4.745482), new Rotation2d(Math.toRadians(60.0)))); // Blue S6
        add(new Pose2d(new Translation2d(5.321046, 4.0259), new Rotation2d(Math.toRadians(0.0)))); // Blue S1
        add(new Pose2d(new Translation2d(4.90474, 3.306318), new Rotation2d(Math.toRadians(300.0)))); // Blue S2

        // DO NOT CHANGE ORDER -- will mess up getDistanceToReefTag()
    }};

    // blah
    // blah.add();
    // aprilTagLocations.add(new Pose2d(5.321046, 4.0259, Math.degreesToRadians(0)));


    private double[][] scoringLocationArray;
    private branchSide branchSide;
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new PoseEstimator. 
     */
    public PoseEstimator(Drivetrain drivetrain, Camera[] cameraArray)
    {
        super("PoseEstimator");
        System.out.println("  Constructor Started:  " + fullClassName);

        this.drivetrain = drivetrain;
        this.gyro = drivetrain.getPigeon2();
        this.cameraArray = cameraArray;

        ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        // This is where the robot starts in AdvantageScope
        poseEstimatorEntry = ASTable.getStructTopic("PoseEstimator", Pose2d.struct).publish();

        double[] doubleArray = {0.0, 0.0, 0.0};

        fillMaps();

        visionStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), doubleArray);
        stateStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), doubleArray);

        configStdDevs();

        if(drivetrain != null && gyro != null)
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

    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPose(pose);
    }

    /**
     * Tells the PoseEstimator how much to trust both the odometry and the vision.  Higher values indicate less trust for either the swerve states or the vision
     */
    public void configStdDevs()
    {
        stateStdDevs.set(0, 0, 0.1); // x in meters
        stateStdDevs.set(1, 0, 0.1); // y in meters
        stateStdDevs.set(2, 0, 0.05); // heading in radians

        visionStdDevs.set(0, 0, 0.15); // x in meters // 0.2
        visionStdDevs.set(1, 0, 0.15); // y in meters // 0.2
        visionStdDevs.set(2, 0, 0.2); // heading in radians // 0.25
    }

    private void fillMaps()
    {
        //Red Left
        leftBranchMap.put(6, new Pose2d( new Translation2d(13.47, 2.75), new Rotation2d(Math.toRadians(-150)))); //S5
        leftBranchMap.put(7, new Pose2d( new Translation2d(14.36, 3.70), new Rotation2d(Math.toRadians(-90)))); //S4
        leftBranchMap.put(8, new Pose2d( new Translation2d(13.98, 5.04), new Rotation2d(Math.toRadians(-30)))); //S3
        leftBranchMap.put(9, new Pose2d( new Translation2d(12.45, 5.20), new Rotation2d(Math.toRadians(30)))); //S2
        leftBranchMap.put(10, new Pose2d( new Translation2d(11.72, 4.00), new Rotation2d(Math.toRadians(90)))); //S1
        leftBranchMap.put(11, new Pose2d( new Translation2d(12.38, 2.90), new Rotation2d(Math.toRadians(150)))); //S6
        //Blue Left
        leftBranchMap.put(17, new Pose2d( new Translation2d(3.60, 3.09), new Rotation2d(Math.toRadians(150)))); //S3 4.073906, 3.306318
        leftBranchMap.put(18, new Pose2d( new Translation2d(3.20, 4.30), new Rotation2d(Math.toRadians(90)))); //S4 3.6576, 4.0259
        leftBranchMap.put(19, new Pose2d( new Translation2d(4.07, 5.30), new Rotation2d(Math.toRadians(30)))); //S5 4.073906, 4.745482
        leftBranchMap.put(20, new Pose2d( new Translation2d(5.12, 5.17), new Rotation2d(Math.toRadians(-30)))); //S6 4.90474, 4.745482
        leftBranchMap.put(21, new Pose2d( new Translation2d(5.79, 4.09), new Rotation2d(Math.toRadians(-90)))); //S1 5.321046, 4.0259
        leftBranchMap.put(22, new Pose2d( new Translation2d(5.18, 2.91), new Rotation2d(Math.toRadians(-150)))); //S2 4.90474, 3.306318
        

        //Red Right
        rightBranchMap.put(6, new Pose2d( new Translation2d(13.67, 2.88), new Rotation2d(Math.toRadians(-150)))); //S5
        rightBranchMap.put(7, new Pose2d( new Translation2d(14.38, 4.00), new Rotation2d(Math.toRadians(-90)))); //S4
        rightBranchMap.put(8, new Pose2d( new Translation2d(13.71, 5.16), new Rotation2d(Math.toRadians(-30)))); //S3
        rightBranchMap.put(9, new Pose2d( new Translation2d(12.68, 5.33), new Rotation2d(Math.toRadians(30)))); //S2
        rightBranchMap.put(10, new Pose2d( new Translation2d(11.72, 4.37), new Rotation2d(Math.toRadians(90)))); //S1
        rightBranchMap.put(11, new Pose2d( new Translation2d(12.12, 3.04), new Rotation2d(Math.toRadians(150)))); //S6
        //Blue Right
        rightBranchMap.put(17, new Pose2d( new Translation2d(3.91, 2.86), new Rotation2d(Math.toRadians(150)))); //S3 4.073906, 3.306318
        rightBranchMap.put(18, new Pose2d( new Translation2d(3.18, 4.03), new Rotation2d(Math.toRadians(90)))); //S4 3.6575, 4.0259
        rightBranchMap.put(19, new Pose2d( new Translation2d(3.80, 5.13), new Rotation2d(Math.toRadians(30)))); //S5 4.073906, 4.745482
        rightBranchMap.put(20, new Pose2d( new Translation2d(5.40, 5.02), new Rotation2d(Math.toRadians(-30)))); //S6 4.90474, 4.745482
        rightBranchMap.put(21, new Pose2d( new Translation2d(6.015, 3.90), new Rotation2d(Math.toRadians(-90)))); //S1 5.321046, 4.0259
        rightBranchMap.put(22, new Pose2d( new Translation2d(4.94, 2.76), new Rotation2d(Math.toRadians(-150)))); //S2 4.90474, 3.306318
        
    }

    /**
     * gets the estimated pose updated by both the odometry and the vision
     * @return estimated pose
     */
    public Pose2d getEstimatedPose()
    {
        if(poseEstimator != null)
        {
            return estimatedPose;
        }
        else
        {
            return new Pose2d();
        }
    }

    public Supplier<Pose2d> getEstimatedPoseSupplier()
    {
        if(poseEstimator != null)
        {
            return () -> estimatedPose;
        }
        else
        {
            return () -> new Pose2d();
        }
    }

    public Pose2d getAprilTagPose(int index)
    {
        return aprilTagLocations.get(index);
    }

    public boolean isReefTag(double tagID)
    {
        if((tagID >= 6 && tagID <= 11) || (tagID >= 17 && tagID <= 22))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public boolean foundReefTag()
    {
        return primaryReefTag != 0;
    }

    public int getPrimaryTagID()
    {
        return primaryReefTag;
    }

    public double getDistanceToReefTag(double tagID)
    {
        if(isReefTag(tagID))
        {
            if(tagID >= 6 && tagID <= 11)
            {
                double x = estimatedPose.getX() - (aprilTagLocations.get((int) tagID - 6).getX());
                double y = estimatedPose.getY() - (aprilTagLocations.get((int) tagID - 6).getY());
                return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            }
            else
            {
                double x = estimatedPose.getX() - (aprilTagLocations.get((int) tagID - 11).getX());
                double y = estimatedPose.getY() - (aprilTagLocations.get((int) tagID - 11).getY());
                return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            }
        }
        else
        {
            return 0.0;
        }
    }

    public BooleanSupplier isReefTagSupplier(double tagID)
    {
        if((tagID >= 6 && tagID <= 11) || (tagID >= 17 && tagID <= 22))
        {
            return () -> true;
        }
        else
        {
            return () -> false;
        }
    }

    /**
     * checks if the pose given is within the field boundaries in meters
     * @param pose
     * @return true or false
     */
    public boolean isPoseInsideField(Pose2d pose)
    {
        if((pose.getX() > -1.0 && pose.getX() < fieldXDimension + 1.0) && (pose.getY() > -1.0 && pose.getY() < fieldYDimension + 1.0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void chooseLocationArray()
    {
        // if(RobotContainer.isRedAllianceSupplier().getAsBoolean())
        // {

        // }

        for(int i = 0; i < 5; i++)
        {
            for(int n = 0; n < 2; n++)
            {
                // scoringLocationArray[i][n] = blueLeftBranchLocationArray[i][n];
            }
        }
    }

    // public double[] chooseClosestBranch(Pose2d aprilTag, boolean isRight)
    // {
    //     scoringLocationArray = (isRight ?  (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? blueRightBranchLocationArray : redRightBranchLocationArray) : (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? blueLeftBranchLocationArray : redLeftBranchLocationArray));
    //     double distance;
    //     double[] closestBranch = {};
    //     double distanceToClosestBranch = Double.MAX_VALUE;
    //     if(isPoseInsideField(aprilTag))
    //     {
    //         for(int b = 0; b < 6; b++)
    //         {
    //             distance = Math.sqrt(Math.pow(scoringLocationArray[b][0] - aprilTag.getX(), 2.0) + Math.pow(scoringLocationArray[b][1] - aprilTag.getY(), 2.0));
    //             if(distance < distanceToClosestBranch)
    //             {
    //                 distanceToClosestBranch = distance;
    //                 closestBranch = scoringLocationArray[b];
    //             }
    //         }
    //     }
    //     return closestBranch;
    // }

    public int getTagIdFromSide()
    {
        if(drivetrain.isRedAllianceSupplier().getAsBoolean())
        {
            if(primaryFace == 1)
                return 10;
            else if(primaryFace == 2)
                return 9;
            else if(primaryFace == 3)
                return 8;
            else if(primaryFace == 4)
                return 7;
            else if(primaryFace == 5)
                return 6;
            else
                return 11;
        }
        else
        {
            if(primaryFace == 1)
                return 21;
            else if(primaryFace == 2)
                return 22;
            else if(primaryFace == 3)
                return 17;
            else if(primaryFace == 4)
                return 18;
            else if(primaryFace == 5)
                return 19;
            else
                return 20;
        }
    }

    public Pose2d closestBranchLocation(Supplier<Integer> aprilTagID, boolean isRight)
    {
        // if(isRight)
        // {
            // return rightBranchMap.getOrDefault(aprilTagID.get(), new Pose2d(2.0, 2.0, new Rotation2d(0.0)));
        // }
        // else
        // {
        //     return leftBranchMap.getOrDefault(aprilTagID.get(), new Pose2d(2.0, 2.0, new Rotation2d(0.0)));
        // }

        Translation2d cwPostTranslation = new Translation2d(0.47, -0.33);
        Translation2d ccwPostTranslation = new Translation2d(0.47, -0.03);
        Rotation2d robotRotation = new Rotation2d(Units.degreesToRadians(-90.0));
        int tag = aprilTagID.get();
        Transform2d leftPostTransform, rightPostTransform;
        Pose2d aprilTagPose, desiredRobotPose;

        if(isReefTag(tag))
        {
            aprilTagPose = aprilTagFieldLayout.getTagPose(tag).get().toPose2d();

            if(tag >= 9 && tag <= 11 || tag >= 20 && tag <= 22)
            {
                if(isRight)
                {
                    rightPostTransform = new Transform2d(cwPostTranslation, robotRotation);
                    desiredRobotPose = aprilTagPose.transformBy(rightPostTransform);
                    return desiredRobotPose;
                }
                else
                {
                    leftPostTransform = new Transform2d(ccwPostTranslation, robotRotation);
                    desiredRobotPose = aprilTagPose.transformBy(leftPostTransform);
                    return desiredRobotPose;
                }
            }
            else
            {
                if(isRight)
                {
                    rightPostTransform = new Transform2d(ccwPostTranslation, robotRotation);
                    desiredRobotPose = aprilTagPose.transformBy(rightPostTransform);
                    return desiredRobotPose;
                }
                else
                {
                    leftPostTransform = new Transform2d(cwPostTranslation, robotRotation);
                    desiredRobotPose = aprilTagPose.transformBy(leftPostTransform);
                    return desiredRobotPose;
                }
            }
        }
        else
        {
            return new Pose2d(2.0, 2.0, new Rotation2d(0.0));
        }
        
    }

    public Pose2d closestBranchLocationSides(boolean isRight)
    {
        Translation2d cwPostTranslation = new Translation2d(0.47, -0.33);
        Translation2d ccwPostTranslation = new Translation2d(0.47, -0.03);
        Rotation2d robotRotation = new Rotation2d(Units.degreesToRadians(-90.0));
        int tag = getTagIdFromSide();
        System.out.println("Target Tag = " + tag);
        Transform2d leftPostTransform, rightPostTransform;
        Pose2d aprilTagPose, desiredRobotPose;

        aprilTagPose = aprilTagFieldLayout.getTagPose(tag).get().toPose2d();

        if(tag >= 9 && tag <= 11 || tag >= 20 && tag <= 22)
        {
            if(isRight)
            {
                rightPostTransform = new Transform2d(cwPostTranslation, robotRotation);
                desiredRobotPose = aprilTagPose.transformBy(rightPostTransform);
                return desiredRobotPose;
            }
            else
            {
                leftPostTransform = new Transform2d(ccwPostTranslation, robotRotation);
                desiredRobotPose = aprilTagPose.transformBy(leftPostTransform);
                return desiredRobotPose;
            }
        }
        else
        {
            if(isRight)
            {
                rightPostTransform = new Transform2d(ccwPostTranslation, robotRotation);
                desiredRobotPose = aprilTagPose.transformBy(rightPostTransform);
                return desiredRobotPose;
            }
            else
            {
                leftPostTransform = new Transform2d(cwPostTranslation, robotRotation);
                desiredRobotPose = aprilTagPose.transformBy(leftPostTransform);
                return desiredRobotPose;
            }
        }
    }

    public boolean getIsRightBranch()
    {
        return isRightBranch;
    }

    private void setPlacingSideToLeft()
    {
        isRightBranch = false;
    }

    private void setPlacingSideToRight()
    {
        isRightBranch = true;
    }

    private int getPlacingFace()
    {
        return primaryFace;
    }

    private void setPlacingFaceToS1()
    {
        primaryFace = 1;
    }

    private void setPlacingFaceToS2()
    {
        primaryFace = 2;
    }

    private void setPlacingFaceToS3()
    {
        primaryFace = 3;
    }

    private void setPlacingFaceToS4()
    {
        primaryFace = 4;
    }

    private void setPlacingFaceToS5()
    {
        primaryFace = 5;
    }

    private void setPlacingFaceToS6()
    {
        primaryFace = 6;
    }
       
    public Command setPlacingSideToLeftCommand()
    {
        return runOnce(() -> setPlacingSideToLeft()).withName("Set Placing Side To Left");
    }

    public Command setPlacingSideToRightCommand()
    {
        return runOnce(() -> setPlacingSideToRight()).withName("Set Placing Side To Right");
    }

    public Command setPlacingFaceToS1Command()
    {
        return runOnce(() -> setPlacingFaceToS1());
    }

    public Command setPlacingFaceToS2Command()
    {
        return runOnce(() -> setPlacingFaceToS2());
    }

    public Command setPlacingFaceToS3Command()
    {
        return runOnce(() -> setPlacingFaceToS3());
    }

    public Command setPlacingFaceToS4Command()
    {
        return runOnce(() -> setPlacingFaceToS4());
    }

    public Command setPlacingFaceToS5Command()
    {
        return runOnce(() -> setPlacingFaceToS5());
    }

    public Command setPlacingFaceToS6Command()
    {
        return runOnce(() -> setPlacingFaceToS6());
    }
        
    

    // public Pose2d closestAprilTag()
    // {
    //     Pose2d closest = Pose2d.nearest(aprilTagLocations);
    // }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    /*
     * This method will be called once per scheduler run
     * Use this for sensors that need to be read periodically.
     * Use this for data that needs to be logged.
     */
    @Override
    public void periodic()
    {
        /*
        if (drivetrain != null && gyro != null && poseEstimator != null) 
        {
            gyroRotation = gyro.getRotation2d();
            swerveModulePositions = drivetrain.getState().ModulePositions;

            // Updates odometry
            poseEstimator.update(gyroRotation, swerveModulePositions);
        }
        */

        for (Camera camera : cameraArray) 
        {
            if (camera != null && drivetrain != null)
            {
                if(gyro != null)
                {
                    // System.out.println("Yaw Log: " + drivetrain.getState().Pose.getRotation().getDegrees());
                    LimelightHelpers.SetRobotOrientation(camera.getCameraName(), drivetrain.getState().Pose.getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
                }

                if(camera.getTagCount() > 0)
                {
                    Pose2d visionPose = camera.getPose();
                    int tagID = (int) camera.getTagId();
                    // variables for pose estimator logic
                    boolean rejectUpdate = false;
                    boolean reefTag = isReefTag(camera.getTagId());
                    double distToTag = camera.avgTagDistance();
                    double robotVelo = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
                    double robotRotation = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);

                    if(visionPose == null)
                    {
                        // System.out.println("Vision Pose equal null");
                        rejectUpdate = true;
                    }

                    if (!reefTag) 
                    {
                        // System.out.println("Not Reef tag");
                        rejectUpdate = true;
                    }

                    if (distToTag > 2.0)
                    {
                        // System.out.println("Distance greater than 2.5");
                        rejectUpdate = true;
                    }

                    // if(!DriverStation.isTeleopEnabled())
                    // {
                    //     rejectUpdate = true;
                    // }

                    if(robotVelo > 2.5)
                    {
                        // System.out.println("robot velo greater than 2.5");
                        rejectUpdate = true;
                    }

                    if(robotRotation > 720.0)
                    {
                        // System.out.println("robot rotation greater than 720");
                        rejectUpdate = true;
                    }

                    if(isReefTag(tagID))
                    {
                        primaryReefTag = tagID;
                        // System.out.println("Primary Tag: " + primaryReefTag);
                    }
                    else
                    {
                        primaryReefTag = 0;
                    }


                    /*
                    if (!rejectUpdate && poseEstimator != null)
                    {
                        // System.out.println("Adding vision measurement for tag: " + tagID);
                        poseEstimator.addVisionMeasurement(
                                visionPose,
                                camera.getTimestamp(),
                                visionStdDevs);

                        // GeneralCommands.setLedCommand(ColorPattern.kSolid, Color.kGreen);
                    }
                    if(rejectUpdate && poseEstimator != null)
                    {
                        // GeneralCommands.setLedCommand(ColorPattern.kSolid, Color.kRed);
                    }
                    else
                    {
                        // GeneralCommands.setLedCommand(ColorPattern.kBlink, Color.kOrange);
                    }
                    */

                    // if any of the conditions above are true, do NOT add the mt2 pose as a vision
                    // measurement
                    if (!rejectUpdate)
                    {
                        primaryReefTag = tagID;
                        drivetrain.addVisionMeasurement(
                                visionPose,
                                camera.getTimestamp(),
                                visionStdDevs);
                    }
                }
                else
                {
                    primaryReefTag = 0;
                }

                // System.out.println(primaryReefTag);
            }
        }

        //OUTPUTS
        if(drivetrain != null && gyro != null && poseEstimator != null)
        {
            // grabs the newest estimated pose
            estimatedPose = drivetrain.getState().Pose; // used to be drivetrain.getPose() <---- CHANGE TOMORROW IF ITS FRICKED UP
            // sets it for advantagescope
            poseEstimatorEntry.set(estimatedPose);
        }
    }

    @Override
    public String toString()
    {
        return "Estimated Pose: " + getEstimatedPose();
    }
}