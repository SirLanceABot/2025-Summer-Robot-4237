package frc.robot.pathplanner;

import java.lang.invoke.MethodHandles;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class PathPlannerLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here

    private static Drivetrain drivetrain;
    private static Field2d field; // object to put on dashboards
    
    private static SendableChooser<Command> autoChooser;
    // private static SendableChooser < Command > leftWall;
    // private static SendableChooser < Command > middle;
    // private static SendableChooser < Command > rightWall;


    private PathPlannerLance()
    {}

    public static void configPathPlanner(RobotContainer robotContainer)
    {
        drivetrain = robotContainer.getDrivetrain();
        // configAutoBuilder();
        configAutoChooser();
        getAutonomousCommand();

        // configPathPlannerLogging();
        
        createEventTriggers();

        // Commands.print("\n\nPLEASE WAIT ...")
        // .andThen(Commands.print("Warming up \"FollowPathCommand\" ..."))
        // .andThen(FollowPathCommand.warmupCommand())
        // .andThen(Commands.print("Warming up\"PathfindingCommand\" ..."))
        // .andThen(PathfindingCommand.warmupCommand())
        // .andThen(Commands.print("DONE warming up paths"))
        // .andThen(Commands.runOnce( () -> configPathPlannerLogging() ).ignoringDisable(true))
        // .schedule();

        // FollowPathCommand.warmupCommand().schedule();
        // PathfindingCommand.warmupCommand().schedule();
    }

    // private static void configAutoBuilder()
    // {
    //     if(drivetrain != null)
    //     {
    //         try
    //         {
    //             RobotConfig config = RobotConfig.fromGUISettings();

    //             AutoBuilder.configure
    //             (
    //                 drivetrain::getPose,
    //                 drivetrain::resetOdometryPose,
    //                 drivetrain::getRobotRelativeSpeeds,
    //                 (speeds, feedforwards) -> setControl(pathApplyRobotSpeeds.withSpeeds(speeds)
    //                                             .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
    //                                             .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
    //                 new PPLTVController(0.02 /*, 3.7*/),
    //                 config,
    //                 shouldFlipPath(),
    //                 drivetrain
    //             );
    //         } 
    //         catch (Exception e) 
    //         {
    //             e.printStackTrace();
    //         }
    //     }
    //     else
    //     {
    //         System.out.println("No Drivetrain");
    //     }
    // }

    /**
     * @return if we should flip out auto pahts based on our alliance
     */
    private static BooleanSupplier shouldFlipPath()
    {
        return 
        () -> 
        {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if(alliance.isPresent()) 
            {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };
    }


    /**
     * Turn on all PathPlanner logging to a Field2d object for NT table "SmartDashboard".
     * <p>PP example from its documentation.
     */
    private static void configPathPlannerLogging()
    {
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback(
            (pose) -> {
                // Do whatever you want with the pose here
                field.setRobotPose(pose);
            }
        );

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
            (pose) -> {
                // Do whatever you want with the pose here
                field.getObject("target pose").setPose(pose);
            }
        );

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback(
            (poses) -> {
                // Do whatever you want with the poses here
                field.getObject("path").setPoses(poses);
            }
        );
    }

    private static void createEventTriggers()
    {
        // // NamedCommands.registerCommand("Intake Algae", GeneralCommands.intakeAlgaeCommand());
        // new EventTrigger("Intake Algae").onTrue(GeneralCommands.intakeAlgaeCommand());

        // // NamedCommands.registerCommand("Score Algae", GeneralCommands.scoreAlgaeCommand());
        // new EventTrigger("Score Algae").onTrue( GeneralCommands.scoreAlgaeCommand());

        // // NamedCommands.registerCommand("Score Coral", GeneralCommands.scoreAlgaeCommand());
        // new EventTrigger("Score Coral").onTrue(GeneralCommands.scoreCoralCommand());

        // // NamedCommands.registerCommand("LED Red", GeneralCommands.setLEDSolid(Color.kRed));
        // new EventTrigger("LED Red").onTrue(GeneralCommands.setLEDSolid(Color.kRed));

        // // NamedCommands.registerCommand("LED BlUE", GeneralCommands.setLEDSolid(Color.kBlue));
        // new EventTrigger("LED BlUE").onTrue(GeneralCommands.setLEDSolid(Color.kBlue));


    }

    private static void configAutoChooser()
    {
        if(AutoBuilder.isConfigured())
        {
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }
        else
        {
            autoChooser = new SendableChooser<Command>();
            autoChooser.setDefaultOption("None", Commands.none());
        }
    }

    public static Command getAutonomousCommand() 
    {
        if(autoChooser != null)
        {
            return autoChooser.getSelected();
        }
        else
        {
            return Commands.none();
        }
        // return new PathPlannerAuto("TEST AUTO - MOVE FORWARD 2M");
    }
}