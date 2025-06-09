package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.PoseEstimator;

public class RobotContainer 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private boolean useFullRobot            = true;

    private boolean useClaw                 = false;
    private boolean useClimb                = false;
    private boolean useDrivetrain           = false;
    private boolean useElevator             = false;
    private boolean useIntake               = false;
    private boolean useIntakeWrist          = false;
    private boolean usePivot                = false;
    private boolean useShuttle              = false;
    // private boolean useLEDs                 = false;

    private boolean useGyro                 = false;
    private boolean usePoseEstimator        = false;
    private boolean useProximity            = false;

    private boolean useScoringSideCamera    = false; // 10.42.37.12 // BLUE CASE
    private boolean useClimbSideCamera      = false; // 10.42.37.11 // CLEAR CASE

    // private boolean useDriverBindings       = false;
    // private boolean useOperatorBindings     = false;
    private boolean useDriverController     = false;
    private boolean useOperatorController   = false;

    //private boolean useAutonomousTab        = false;

    // public final boolean fullRobot;

    private final Claw claw;
    private final Climb climb;
    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Intake intake;
    private final IntakeWrist intakeWrist;
    private final Pivot pivot;
    private final LEDs leds;

    private final Camera[] cameraArray = new Camera[2];
    private final PoseEstimator poseEstimator;
    private final Proximity intakeProximity;
    private final Proximity shooterProximity;
    private final Proximity backupShooterProximity;


    // private final DriverButtonBindings driverButtonBindings;
    // private final OperatorButtonBindings operatorButtonBindings;
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    //public final AutonomousTab autonomousTab;
    

    /** 
     * The container for the robot. Contains subsystems, OI devices, and commands.
     * Uses the default access modifier so that the RobotContainer object can only be constructed in this same package.
     */
    RobotContainer()
    {
        // fullRobot = useFullRobot;

        claw = (useFullRobot || useClaw)
                ? new Claw()
                : null;

        climb = (useFullRobot || useClimb)
                ? new Climb()
                : null;

        drivetrain = (useFullRobot || useDrivetrain)
                ? TunerConstants.createDrivetrain()
                : null;

        elevator = (useFullRobot || useElevator)
                ? new Elevator()
                : null;

        intake = (useFullRobot || useIntake)
                ? new Intake()
                : null;

        intakeWrist = (useFullRobot || useIntakeWrist)
                ? new IntakeWrist()
                : null;

        pivot = (usePivot)
                ? new Pivot()
                : null;

        // leds = (useFullRobot || useLEDs)
        //         ? new LEDs()
        //         : null;

        leds = new LEDs();

        intakeProximity = (useFullRobot || useProximity)
                ? new Proximity(Constants.Proximity.CORAL_INTAKE_PORT)
                : null;

        shooterProximity = (useFullRobot || useProximity)
                ? new Proximity(Constants.Proximity.SHOOTER_PORT)
                : null;

        backupShooterProximity = (useFullRobot || useProximity)
                ? new Proximity(Constants.Proximity.BACKUP_SHOOTER_PORT)
                : null;

        cameraArray[0] = (useScoringSideCamera)
                ? new Camera("limelight-scoring")
                : null;

        cameraArray[1] = (useFullRobot || useClimbSideCamera)
                ? new Camera("limelight-climb")
                : null;

        poseEstimator = (useFullRobot || usePoseEstimator)
                ? new PoseEstimator(drivetrain, cameraArray)
                : null;

        driverController = (useFullRobot || useDriverController)
                ? new CommandXboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT)
                : null;

        operatorController = (useFullRobot || useOperatorController)
                ? new CommandXboxController(Constants.Controllers.OPERATOR_CONTROLLER_PORT)
                : null; 


        
        // operatorButtonBindings = (useFullRobot || useBindings)
        // ? new OperatorButtonBindings(this)
        // : null;

        // driverButtonBindings = (useFullRobot || useBindings)
        // ? new DriverButtonBindings(this)
        // : null;
    }

    public boolean useFullRobot()
    {
        return useFullRobot;
    }
    

    public Claw getClaw()
    {
        return claw;
    }

    public Climb getClimb()
    {
        return climb;
    }

    public Drivetrain getDrivetrain()
    {
        return drivetrain;
    }

    public Elevator getElevator()
    {
        return elevator;
    }

    public Intake getIntake()
    {
        return intake;
    }

    public IntakeWrist getIntakeWrist()
    {
        return intakeWrist;
    }

    public Pivot getPivot()
    {
        return pivot;
    }

    public PoseEstimator getPoseEstimator()
    {
        return poseEstimator;
    }

    public Proximity getIntakeProximity()
    {
        return intakeProximity;
    }

    public Proximity getShooterProximity()
    {
        return shooterProximity;
    }

    public Proximity getBackupShooterProximity()
    {
        return backupShooterProximity;
    }

    public CommandXboxController getDriverController()
    {
        return driverController;
    }

    public CommandXboxController getOperatorController()
    {
        return operatorController;
    }

    public LEDs getLEDs()
    {
        return leds;
    }

    public Camera getScoringSideCamera()
    {
        return cameraArray[0];
    }

    public Camera getClimbSideCamera()
    {
        return cameraArray[1];
    }

    public Command getAutonomousCommand() 
    {
        return Commands.print("No autonomous command configured");
    }
}
