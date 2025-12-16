package frc.robot.sensors;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Use this class as a template to create other sensors.
 */
public class MagnetLimitSwitch extends SensorLance
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
    private final DigitalInput swyftMagnetLimitSwitch = new DigitalInput(0);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public MagnetLimitSwitch()
    {   
        super("Magnet Limit Switch");
        System.out.println("  Constructor Started:  " + fullClassName);



        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    public boolean isDetected()
    {
        return !swyftMagnetLimitSwitch.get();
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        System.out.println("Is object detected: " + isDetected());
    }

    @Override
    public String toString()
    {
        return "Swyft magnet limit switch :)";
    }
}