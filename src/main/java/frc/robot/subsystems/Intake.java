package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.SparkMaxLance;

/**
 * This is the Intake subsystem
 * @author Tanishka
 */
public class Intake extends SubsystemLance 
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
    /** 
     * This class sets the direction the motor is going in
     */
 
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    // private final TalonFXLance motor = new TalonFXLance(Constants.Intake.MOTOR_PORT, Constants.Intake.MOTOR_CAN_BUS, "Intake Motor");
    private final SparkMaxLance motor = new SparkMaxLance(Constants.Intake.MOTOR_PORT, Constants.Intake.MOTOR_CAN_BUS, "Intake Motor");
    //private final TalonFXLance bottomMotor = new TalonFXLance(12, Constants.ROBORIO, "Bottom Motor");

    // private final double PERCENT_VOLTAGE = 0.9;
    // private final double VOLTAGE = PERCENT_VOLTAGE * Constants.END_OF_MATCH_BATTERY_VOLTAGE;

    private double rollerPosition = 0.0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Creates a new Intake.
     */
    public Intake() 
    {
        super("Intake");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        // SendableRegistry.addLW(this, "Intake", "MY Subsystem");
        // addChild("Motor 1", motor1);
        // addChild("Motor 2", motor2);

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors() 
    {
        // Factory Defaults
        motor.setupFactoryDefaults();
        //bottomMotor.setupFactoryDefaults();
        // Do Not Invert Motor Direction
        motor.setupInverted(true); // set to true
        // Set Coast Mode
        motor.setupBrakeMode();

        // motor.setupVelocityConversionFactor(RPM_TO_FPS);

        // motor.setupCurrentLimit(30.0, 50.0, 0.5); //TODO: Check neo550 current
        motor.setSafetyEnabled(false);
        //bottomMotor.setSafetyEnabled(false);
    }

    // public double getPosition() 
    // {
    //     return rollerPosition;
    // }

    // public double getVelocity() 
    // {
    //     return rollerVelocity;
    // }

    public void set(double speed) 
    {
        motor.set(speed);
        System.out.println("Intake amps: " + motor.getCurrentAmps());
    }

    private void setVoltage(double voltage) 
    {
        motor.setVoltage(voltage);
    }

    public void pickupCoral() 
    {
        motor.setupCoastMode();
        set(0.9);
    }

    public void ejectCoral() 
    {
        motor.setupCoastMode();
        set(-0.2);
    }

    public BooleanSupplier isCoralOrAlgaeIn()
    {
        return () -> motor.getCurrentAmps() >= 10;
    }

    public BooleanSupplier isAlgaeIn()
    {
        return () -> motor.getCurrentAmps() >= 50;
    }

    public void pulse()
    {
        // motor.setupBrakeMode();
        if(motor.getCurrentAmps() > 1.0)
        {
            motor.set(0.0);
        }
        else
        {
            motor.set(-0.35);
        }
        // System.out.println("Intake amps: " + motor.getCurrentAmps());
    }

    public void pickupAlgae()
    {
        motor.setupBrakeMode();
        set(-0.5);
    }

    public void ejectAlgae()
    {
        motor.setupCoastMode();
        set(1.0);
    }

    // public void pulse()
    // {
    //     for(int i = 0; )
    // }

    public void stop() 
    {
        motor.set(0.0);
    }

    public Command pickupCoralCommand() 
    {
        return run(() -> pickupCoral()).withName("Pickup Coral");
    }

    public Command ejectCoralCommand() 
    {
        return run(() -> ejectCoral()).withName("Eject Coral");
    }

    public Command pulseCommand()
    {
        return run(() -> pulse());
    }

    public Command pickupAlgaeCommand()
    {
        return run(() -> pickupAlgae()).withName("Pickup Algae");
    }

    public Command ejectAlgaeCommand()
    {
        return runOnce(() -> ejectAlgae()).withName("Eject Algae");
    }

    public Command stopCommand() 
    {
        return runOnce(() -> stop()).withName("Stop");
    }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        //rollerPosition = motor.getPosition();
        //periodicData.bottomRollerPosition = bottomMotor.getPosition();
        //rollerVelocity = motor.getVelocity();
        //periodicDatopRollerVelocityta.bottomRollerVelocity = bottomMotor.getVelocity();
        // System.out.println("Intake amps: " + motor.getCurrentAmps());
    }

    @Override
    public String toString() 
    {
        return "Current Intake Position: " + rollerPosition;
    }
}