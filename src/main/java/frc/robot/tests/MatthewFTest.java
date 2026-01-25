package frc.robot.tests;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.motors.TalonFXLance;
import frc.robot.sensors.CANRange;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.IntakeWrist.Position;

@SuppressWarnings("unused")
public class MatthewFTest implements Test
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

    // private final ExampleSubsystem exampleSubsystem;
    private final Joystick joystick = new Joystick(0);
    // private final TalonFXLance motor1 = new TalonFXLance(1, "rio", "Test Motor 1");
    private final Drivetrain drivetrain;
    private final DoubleSupplier start = () -> (0);
    private final DoubleSupplier twoRadians = () ->(Math.PI / 4);
    // private final Intake intake;
    // private final IntakeWrist intakeWrist;
    


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public MatthewFTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.getDrivetrain();
        configJoystick();
        // this.intake = robotContainer.getIntake();
        // this.intakeWrist = robotContainer.getIntakeWrist();
        
        // this.exampleSubsystem = robotC ontainer.exampleSubsystem;

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    private void configJoystick()
    {
        // Trigger dpadUp = joystick.povUp();

    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

        

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {
        // drivetrain.getPigeon2().reset();
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        if(joystick.getRawButton(1))
        {
            drivetrain.angleLockDriveCommand(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () -> 0.1, twoRadians).schedule();
        }
        else
        {
            drivetrain.angleLockDriveCommand(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () -> 0.1, start).schedule();
        }

        System.out.println("Angle: " + drivetrain.getPigeon2().getRotation2d().getDegrees());
       
        // if (joystick.getRawButton(1) && (canRange.getDistanceSupplier().getAsDouble() > 0.1 && canRange.getDistanceSupplier().getAsDouble() < 0.3))
        // {
        //     motor1.set(0.1);
        // }
        // else if (joystick.getRawButton(1) && (canRange.getDistanceSupplier().getAsDouble() >= 0.2 && canRange.getDistanceSupplier().getAsDouble() < 0.5))
        // {
        //     motor1.set(0.5);
        // }
        // else
        // {
        //     motor1.stopMotor();
        // }
        /* 
        else if (joystick.getRawButton(2))
        {
            // intakeWrist.moveToSetPositionCommand(Position.kManipAlgaePosition).schedule();
            // intake.ejectCoralCommand().schedule();
        }
        else if(joystick.getRawButton(4))
        {
            // intakeWrist.moveToSetPositionCommand(Position.kRestingPosition).schedule();
        }
        else
        {
            // intakeWrist.stopCommand().schedule();
        } 

        if(joystick.getRawButton(5))
        {
            // intake.pickupCoralCommand().schedule();
        }
        else if(joystick.getRawButton(6))
        {
            // intake.ejectCoralCommand().schedule();
        }
        else
        {
            // intake.stopCommand().schedule();
        }

        // System.out.println(intakeWrist.toString());
        */
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {
        // intakeWrist.stopCommand().schedule();
        // intake.stopCommand().schedule();
    } 
}

