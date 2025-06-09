
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 * @author Matthew Fontecchio
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem 
{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private static SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    //Used for logging odometry
    private final NetworkTable ASTable;
    private StructPublisher<Pose2d> drivetrainEntry;
    private Pose2d estimatedPose = new Pose2d();


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private Field2d field = new Field2d();

    //Drive the robot
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MaxDriveSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.RobotCentric autoDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    //Lock the wheels
    public static final SwerveRequest.SwerveDriveBrake lock = new SwerveRequest.SwerveDriveBrake();

    //Point the wheels at a specific angle
    private static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private static final SwerveRequest.FieldCentricFacingAngle angleLockDrive = new SwerveRequest.FieldCentricFacingAngle();


    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Drivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) 
    {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }

        ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        drivetrainEntry = ASTable.getStructTopic("DrivetrainOdometry", Pose2d.struct).publish();

        configAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Drivetrain(SwerveDrivetrainConstants drivetrainConstants,double odometryUpdateFrequency,SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }

        ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        drivetrainEntry = ASTable.getStructTopic("DrivetrainOdometry", Pose2d.struct).publish();

        configAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Drivetrain(SwerveDrivetrainConstants drivetrainConstants,double odometryUpdateFrequency,Matrix<N3, N1> odometryStandardDeviation,Matrix<N3, N1> visionStandardDeviation,SwerveModuleConstants<?, ?, ?>... modules) 
    {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }
        SmartDashboard.putData("Swerve/Field", field);

        ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        drivetrainEntry = ASTable.getStructTopic("DrivetrainOdometry", Pose2d.struct).publish();

        configAutoBuilder();
    }

    private void configAutoBuilder()
    {
        try
        {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure
            (
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(pathApplyRobotSpeeds.withSpeeds(speeds)
                                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    new PIDConstants(9, 0, 0),
                    new PIDConstants(8.5, 0, 0) // used to be 7
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } 
        catch (Exception e) 
        {
            e.printStackTrace();
        }
    }

    public Command removeAlgaeCommand()
    {
        return runOnce(() -> setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.0, 1.0, 0.0))));
    }

    public Command stopCommand()
    {
        return runOnce(() -> setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))));
    }

    // private Rotation2d angleToNearestBranch()
    // {
    //     double[] nearestBranch = poseEstimator.chooseClosestBranch();
    //     Rotation2d targetDirection = new Rotation2d((nearestBranch[2] / 360) * 2 * Math.PI);
    //     return targetDirection; //returns the angle to the nearest branch in radians
    // }

    // private DoubleSupplier xSpeedToNearestBranch()
    // {
    //     double originalXDistance = poseEstimator.chooseClosestBranch()[0] - poseEstimator.getEstimatedPose().getX();
    //     DoubleSupplier xDistance = () -> poseEstimator.chooseClosestBranch()[0] - poseEstimator.getEstimatedPose().getX();
    //     DoubleSupplier yDistance = () -> poseEstimator.chooseClosestBranch()[1] - poseEstimator.getEstimatedPose().getY();
    //     //Calculates the speed to move in the X direction based on how far away you are from the desired position on the reef
    //     return () -> (xDistance.getAsDouble() > yDistance.getAsDouble() ? (1.0) * (xDistance.getAsDouble() / originalXDistance): (xDistance.getAsDouble()/yDistance.getAsDouble()) * (xDistance.getAsDouble() / originalXDistance)); //TODO Might need to increase the multiplier when close to the desired position
    // }

    // private DoubleSupplier ySpeedToNearestBranch()
    // {
    //     double originalXDistance = poseEstimator.chooseClosestBranch()[0] - poseEstimator.getEstimatedPose().getX();
    //     DoubleSupplier xDistance = () -> poseEstimator.chooseClosestBranch()[0] - poseEstimator.getEstimatedPose().getX();
    //     DoubleSupplier yDistance = () -> poseEstimator.chooseClosestBranch()[1] - poseEstimator.getEstimatedPose().getY();

    //     //Calculates the speed to move in the Y direction based on how far away you are from the desired position on the reef
    //     return () -> (yDistance.getAsDouble() > xDistance.getAsDouble() ? (1.0) * (xDistance.getAsDouble() / originalXDistance): (yDistance.getAsDouble()/xDistance.getAsDouble()) * (xDistance.getAsDouble() / originalXDistance)); //TODO Might need to increase the multiplier when close to the desired position
    // }

    // public PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    //TODO send this to a new autobuilder to run the path
    //Creates a path to run to get from one spot on the field to another
    // private PathPlannerPath createOnTheFlyPath(Pose2d... pose)
    // {
    //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pose);
    //     PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0,pose[pose.length - 1].getRotation()));
    //     return path;
    // }

    public Pose2d getPose()
    {
        return getState().Pose;
        
    }

  

    /**
     * resets odometry by reseting the gyro, pose, and the left / right motors
     */
    // public void resetOdometry()
    // {
    //     odometry.resetPosition(
    //         gyro.getRotation2d(),
    //         leftLeaderPosition,
    //         rightLeaderPosition,
    //         pose
    //     );
    // }

    /**
     * @return
     * the robot's estimated chassis speed based on the left and right velocities
     */
    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return getState().Speeds;
    }

    /**
     * @param chassisSpeeds
     * the robot's chassis speed
     */
    public Command driveRobotRelative(ChassisSpeeds chassisSpeeds)
    {
        ChassisSpeeds wheelSpeeds = getState().Speeds;
        SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(.12,12.0/3.7);


        double xDirectionWheelSpeedInVolts = 0.0;// motorFeedforward.calculate(wheelSpeeds.vxMetersPerSecond);
        double yDirectionWheelSpeedInVolts = 0.0;//motorFeedforward.calculate(wheelSpeeds.vyMetersPerSecond);
        // System.out.println("-------------------LV = " + leftWheelSpeedInVolts + ", RV = " + rightWheelSpeedInVolts + ", RV = " + rightLeaderVelocity + ", LV = " + leftLeaderVelocity + ", CS = " + chassisSpeeds);
        SmartDashboard.putString("Chassis Speeds", chassisSpeeds.toString());
        SmartDashboard.putNumber("X Volts", xDirectionWheelSpeedInVolts);
        SmartDashboard.putNumber("Y Volts", yDirectionWheelSpeedInVolts);
        SmartDashboard.putNumber("X velocity", xDirectionWheelSpeedInVolts);
        SmartDashboard.putNumber("Y velocity", yDirectionWheelSpeedInVolts);

        return applyRequest(() -> autoDrive.withVelocityX(chassisSpeeds.vxMetersPerSecond)
                                    .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                                    .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));

    }
    

    public void resetOdometryPose(Pose2d pose)
    {
        resetPose(pose);
    }

   


    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) 
    {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns a command that will lock the wheels of the drivetrain in an X shape
     * 
     * @return Command to lock wheels
     * @author Matthew Fontecchio
     */
    public Command lockWheelsCommand()
    {
        return applyRequest(() -> lock);
    }

    /**
     * Returns a command that will drive the robot
     * The current drive speed is 1/4 that of the max for testing
     * @param leftYAxis the left Y axis of the controller
     * @param leftXAxis the left X axis of the controller
     * @param rightXAxis the right X axis of the controller
     * @return Command to Drive
     * @author Matthew Fontecchio
     */
    public Command driveCommand(DoubleSupplier leftYAxis, DoubleSupplier leftXAxis, DoubleSupplier rightXAxis, DoubleSupplier setScaleFactor)
    {
        return applyRequest(
            () -> drive
                .withVelocityX((leftYAxis.getAsDouble() * (TunerConstants.MaxDriveSpeed * setScaleFactor.getAsDouble())) / 1.0)
                .withVelocityY((leftXAxis.getAsDouble() * (TunerConstants.MaxDriveSpeed * setScaleFactor.getAsDouble())) / 1.0)
                .withRotationalRate((rightXAxis.getAsDouble() * (TunerConstants.MaxAngularRate * setScaleFactor.getAsDouble())) / 1.0)
        );
    }

    /**
     * Returns a command that will point the robot's wheels in a specified direction
     * 
     * @param leftYAxis the left Y axis of the controller
     * @param leftXAxis the left X axis of the controller
     * @return Command to point the wheels at a specific angle
     * @author Matthew Fontecchio
     */
    public Command pointCommand(DoubleSupplier leftYAxis, DoubleSupplier leftXAxis)
    {
        return applyRequest(
            () -> point
                .withModuleDirection(new Rotation2d(leftYAxis.getAsDouble(), leftXAxis.getAsDouble()))

        );
    }

    public BooleanSupplier isRedAllianceSupplier()
    {
        return () ->
        {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
                System.out.println("Alliance = " + alliance.get());
                return alliance.get() == DriverStation.Alliance.Red;
            }
            DriverStation.reportError("No alliance is avaliable, assuming Blue", false);
            return false;
        };
    }

    public void setYaw()
    {
        if(DriverStation.getAlliance().isPresent())
        {
            if(DriverStation.Alliance.Blue == DriverStation.getAlliance().get())
            {
                getPigeon2().setYaw(180.0);
                System.out.println("Setting to 0.0");
                DriverStation.getAlliance().ifPresent(allianceColor -> {
                    setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                            ? kRedAlliancePerspectiveRotation
                            : kBlueAlliancePerspectiveRotation);}
                    );
            }
            else
            {
                getPigeon2().setYaw(0.0);
                System.out.println("Setting to 180");
            } // TODO: TEST THIS (REMOVE IF WORKS)
        }
    }


    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) 
    {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) 
    {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // public Command driveBackToRemoveAlgaeCommand()
    // {

    // }

    @Override
    public void periodic() 
    {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) 
        {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        //logs position data
        estimatedPose = getState().Pose;
        drivetrainEntry.set(estimatedPose);
    }

    private void startSimThread() 
    {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

}
