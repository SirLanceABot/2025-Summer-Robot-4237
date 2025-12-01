package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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
    
    private double armAngle = 0.0;
    private double armKp = 4237.0;

    // gearbox?

    private final PIDController veryGoodController = new PIDController(armKp, 0, 0);
    private final Encoder encoder = 
        new Encoder(4237, 4237);
    private final TalonFXLance motor1 = new TalonFXLance(0, Constants.ROBORIO, "motor1");

    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mech.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d armTower =
        armPivot.append(new MechanismLigament2d("ArmTower", 30, -45));
    private final MechanismLigament2d arm =
        armPivot.append(
            new MechanismLigament2d(
                "Arm",
                30,
                armAngle,
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

        // configMotors();

        SmartDashboard.putData("Arm Sim", mech);
        armTower.setColor(new Color8Bit(Color.kBlue));

        // LoggerMechanism2d mechanism = new LoggedMechanism2d(3,3);
        // Logger.recordOutput("MyMechanism", mechanism);

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.
        
        
        SmartDashboard.putNumber("Value", 1);
        System.out.println("please work--------------------------------------------------------------------------------------------");
    }

    @Override
    public String toString()
    {
        return "Kabe Lame Mechanism";
    }
}
