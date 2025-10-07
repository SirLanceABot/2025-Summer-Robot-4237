package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class Sonic extends SubsystemLance
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
    private final TalonFXLance firstMotor = new TalonFXLance(0, Constants.CANIVORE, "First");
    private final TalonFXLance secondMotor = new TalonFXLance(0, Constants.CANIVORE, "Second");

    private double firstPos = 0.0;
    private double secondPos = 0.0;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public Sonic()
    {
        super("Sonic");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        firstMotor.setupFactoryDefaults();
        secondMotor.setupFactoryDefaults();

        firstMotor.setupBrakeMode();
        secondMotor.setupBrakeMode();

        firstMotor.setupInverted(false);
        secondMotor.setupInverted(false);
    }

    // Use a method reference instead of this method
    // public Command stopCommand()
    // {
    //     return run( () -> stop() );
    // }

    public double getFirstPos()
    {
        return firstPos;
    }

    public double getSecondPos()
    {
        return secondPos;
    }

    public void setFirst(double speed)
    {
        firstMotor.set(speed);
    }

    public void setSecond(double speed)
    {
        secondMotor.set(speed);
    }

    public void firstOn()
    {
        firstMotor.set(0.5);
    }

    public void secondOn()
    {
        secondMotor.set(0.5);
    }

    public void off()
    {
        firstMotor.set(0.0);
        secondMotor.set(0.0);
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        firstPos = firstMotor.getPosition();
        secondPos = secondMotor.getPosition();
    }

    @Override
    public String toString()
    {
        return "";
    }
}
