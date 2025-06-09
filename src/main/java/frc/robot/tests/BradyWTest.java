package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

@SuppressWarnings("unused")
public class BradyWTest implements Test
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



    // *** CLASS & INSTANCE VARIABLES ***
    // Put all class and instance variables here.
    private final RobotContainer robotContainer;

    private final Elevator elevator;
    private final Joystick joystick = new Joystick(0);
    private final Pivot pivot;
    // private final ExampleSubsystem exampleSubsystem;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public BradyWTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        elevator = robotContainer.getElevator();
        pivot = robotContainer.getPivot();
        // this.exampleSubsystem = robotContainer.exampleSubsystem;

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

        

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {}

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // if(joystick.getRawButton(1))
        // {
        //     elevator.moveToSetPosition(Constants.TargetPosition.kL3);
        // }
        // else if(joystick.getRawButton(2))
        // {
        //     elevator.moveToSetPosition(Constants.TargetPosition.kL2);
        // }
        // else 
        // {
        //     elevator.stop();
        // }

        // if(joystick.getRawButton(1))
        // {
        //     pivot.set(0.5);
        // }
        // else if(joystick.getRawButton(2))
        // {
        //     pivot.on(-0.5);
        // }
        if(joystick.getRawButton(3))
        {
            pivot.onCommand(0.5).schedule();
        }
        else if(joystick.getRawButton(4))
        {
            pivot.onCommand(-0.5).schedule();
        }
        else 
        {
            pivot.stopCommand().schedule();
        }

        // if (joystick.getRawButton(1))
        // {
        //     pivot.moveToSetPositionCommand(Constants.TargetPosition.kL1).schedule();
        // }
        // else if(joystick.getRawButton(2))
        // {
        //     pivot.moveToSetPositionCommand(Constants.TargetPosition.kL2).schedule();
        // }
        // else
        // {
        //     pivot.holdCommand().schedule();
        // }


        System.out.println("Elevator Position: " + elevator.getPosition());
    }

    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
