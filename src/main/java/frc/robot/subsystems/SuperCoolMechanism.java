package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;



/**
 * This is an example of what a subsystem should look like.
 */
public class SuperCoolMechanism extends SubsystemLance
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

    private double armKp = Constants.SuperCoolMechanism.kDefaultArmKp;
    private double armSetpointDegrees = Constants.SuperCoolMechanism.kDefaultArmSetpointDegrees;

    private final DCMotor armGearbox = DCMotor.getVex775Pro(2);

    // gearbox?

    private final PIDController veryGoodController = new PIDController(armKp, 0, 0);
    private final Encoder encoder = 
        new Encoder(Constants.SuperCoolMechanism.kEncoderAChannel, Constants.SuperCoolMechanism.kEncoderBChannel);
    private final TalonFXLance motor1 = new TalonFXLance(0, Constants.ROBORIO, "motor1");

    private final SingleJointedArmSim armSim = 
        new SingleJointedArmSim(
            armGearbox,
            Constants.SuperCoolMechanism.kArmReduction,
            SingleJointedArmSim.estimateMOI(Constants.SuperCoolMechanism.kArmLength,Constants.SuperCoolMechanism.kArmMass),
            Constants.SuperCoolMechanism.kArmLength,
            Constants.SuperCoolMechanism.kMinAngleRads,
            Constants.SuperCoolMechanism.kMaxAngleRads,
            true,
            0,
            Constants.SuperCoolMechanism.kArmEncoderDistPerPulse,
            0.0
            );
    private final EncoderSim encoderSim = new EncoderSim(encoder);

    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mech.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d armTower =
        armPivot.append(new MechanismLigament2d("ArmTower", 30, -45));
    private final MechanismLigament2d arm =
        armPivot.append(
            new MechanismLigament2d(
                "Arm",
                30,
                Units.radiansToDegrees(armSim.getAngleRads()),
                6,
                new Color8Bit(Color.kYellow)));

    

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Kabe lame Mechanism. 
     */
    public SuperCoolMechanism()
    {
        super("Kabe Lame Mechanism");
        System.out.println("Constructor Started:  " + fullClassName);

        encoder.setDistancePerPulse(Constants.SuperCoolMechanism.kArmEncoderDistPerPulse);

        configMotors();

        SmartDashboard.putData("Mechanism2D", mech);
        armTower.setColor(new Color8Bit(Color.kBlue));

        System.out.println("value placed");

        Preferences.initDouble("Arm Position", armSetpointDegrees);
        Preferences.initDouble("Arm P", armKp);

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor1.setupFactoryDefaults();

        motor1.setupBrakeMode();

        motor1.setupInverted(false);
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    

    public void simulationPeriodic()
    {
        armSim.setInput(motor1.get() * RobotController.getBatteryVoltage());

        armSim.update(0.020);

        encoderSim.setDistance(armSim.getAngleRads());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps())
        );

        arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }

    public void loadPreferences()
    {
        armSetpointDegrees = Preferences.getDouble("Arm Position", armSetpointDegrees);
        if(armKp != Preferences.getDouble("Arm P", armKp)) 
        {
            armKp = Preferences.getDouble("Arm P", armKp);
            veryGoodController.setP(armKp);
        }
    }

    public void reachSetpoint()
    {
        var pidOutput =
            veryGoodController.calculate(
                encoder.getDistance(), Units.degreesToRadians(armSetpointDegrees));
            motor1.setVoltage(pidOutput);

    }

    public void stop()
    {
        motor1.set(0.0);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        simulationPeriodic();

        SmartDashboard.putNumber("Value", 1);

    }

    @Override
    public String toString()
    {
        return "Kabe Lame Mechanism";
    }
}
