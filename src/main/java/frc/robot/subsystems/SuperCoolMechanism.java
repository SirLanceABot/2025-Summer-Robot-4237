package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
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

    // private final Elevator elevator;
    private final TalonFXLance motor1 = new TalonFXLance(4, Constants.ROBORIO, "Motor 1");
    private final TalonFXLance motor2 = new TalonFXLance(12, Constants.ROBORIO, "Motor 2");

    
    
    private final MechanismLigament2d armStageOne;
    private final MechanismLigament2d armStageTwo;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new SuperCoolMechanism. 
     */
    public SuperCoolMechanism()
    {
        super("Super Cool Mechanism");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        Mechanism2d arm = new Mechanism2d(3,3);
        MechanismRoot2d root = arm.getRoot("arm", 2, 0);
    
        armStageOne = root.append(new MechanismLigament2d("arm stage one", 1, 90));
        armStageTwo = armStageOne.append(new MechanismLigament2d("arm stage two", 1, 90));

        SmartDashboard.putData("Mech2d", arm);

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor1.setupFactoryDefaults();
        motor2.setupFactoryDefaults();
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        motor1.set(speed);
        motor2.set(speed);
    }

    public void stop()
    {
        motor1.set(0.0);
        motor2.set(0.0);
    }

    public Command onCommand()
    {
        return run( () -> set(0.25) );
    }

    public Command setCommand(DoubleSupplier speed)
    {
        return run( () -> set(MathUtil.clamp(speed.getAsDouble(), 0.0, 0.5)) );
    }

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

         
    }

    @Override
    public String toString()
    {
        return "Mechanism";
    }
}
