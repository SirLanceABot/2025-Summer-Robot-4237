package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;

/**
 * Creates a new climb subsystem with one Kraken motor
 * @author Logan Bellinger
 * @author Mason Bellinger
 */
public class Climb extends SubsystemLance
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
    public enum Position
    {
        kClimbUpCagePosition(Constants.Climb.CLIMB_UP_CAGE_POSITION),
        kClimbDownCagePosition(Constants.Climb.CLIMB_DOWN_CAGE_POSITION);

        double value;

        private Position(double value)
        {
            this.value = value;
        }
    }

    

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final TalonFXLance leadMotor = new TalonFXLance(Constants.Climb.LEAD_MOTOR_PORT, Constants.Climb.MOTOR_CAN_BUS, "Climb Lead Motor");
    private final TalonFXLance followMotor = new TalonFXLance(Constants.Climb.FOLLOW_MOTOR_PORT, Constants.Climb.MOTOR_CAN_BUS, "Climb Follow Motor");

    private final double POSITION_TOLERANCE = 1.0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Climb. 
     */
    public Climb()
    {
        super("Climb");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        // SendableRegistry.addLW(this, "Example Subsystem", "MY Subsystem");
        // addChild("Motor 1", motor1);
        // addChild("Motor 2", motor2);

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    /**
     * Puts motor in brake mode and sets up soft and hard limit switches
     */
    private void configMotors()
    {
        leadMotor.setupFactoryDefaults();
        followMotor.setupFactoryDefaults();
        leadMotor.setSafetyEnabled(false);
        followMotor.setSafetyEnabled(false);
        leadMotor.setupBrakeMode();
        followMotor.setupBrakeMode();
        leadMotor.setupInverted(true);

        followMotor.setupFollower(Constants.Climb.LEAD_MOTOR_PORT, false);

        leadMotor.setPosition(0.0);
        followMotor.setPosition(0.0);
        
        // motor.setupPIDController(0, 0.0, 0.0, 0.0);

        // leadMotor.setupForwardSoftLimit(FORWARD_SOFT_LIMIT, true);
        // leadMotor.setupReverseSoftLimit(REVERSE_SOFT_LIMIT, true);
        // motor.setupForwardHardLimitSwitch(true, true);
        // motor.setupReverseHardLimitSwitch(true, true);
        leadMotor.setupForwardHardLimitSwitch(true, true);
        leadMotor.setupReverseHardLimitSwitch(true, true);

    }

    /**
     * gets the encoder position of the motor
     * @return motor position
     */
    public double getPosition()
    {
        return leadMotor.getPosition();
    }

    /**
     * sets the motor to the speed passed to the function
     * @param speed sets speed from -1.0 to 1.0
     */
    public void set(double speed)
    {
        leadMotor.set(MathUtil.clamp(speed, -1, 1));
    }

    /**
     * Moves the climb so the robot will climb up the cage
     */
    public void climbUpCage()
    {
        set(1.0);
    }

    /**
     * Moves the climb so the robot will climb down the cage
     */
    public void climbDownCage()
    {
        set(-1.0);
    }

    /**
     * Moves the climb to the position where the robot will be hanging from the cage
     */
    public void climbToUpPosition()
    {
        // if(position < (Constants.Climb.CLIMB_UP_CAGE_POSITION - POSITION_TOLERANCE))
        // {
        //     climbUpCage();
        // }
        // else if(position > (Constants.Climb.CLIMB_UP_CAGE_POSITION + POSITION_TOLERANCE))
        // {
        //     climbDownCage();
        // }
        // else
        // {
        //     set(0.0);
        // }

        // position = motor.getPosition();
        leadMotor.setControlPosition(Constants.Climb.CLIMB_UP_CAGE_POSITION);
    }

    /**
     * Moves the climb to the position where the robot will be ready to climb
     */
    public void climbToDownPosition()
    {
        // if(position < (Constants.Climb.CLIMB_DOWN_CAGE_POSITION - POSITION_TOLERANCE))
        // {
        //     climbUpCage();
        // }
        // else if(position > (Constants.Climb.CLIMB_DOWN_CAGE_POSITION + POSITION_TOLERANCE))
        // {
        //     climbDownCage();
        // }
        // else
        // {
        //     set(0.0);
        // }

        // position = motor.getPosition();
        leadMotor.setControlPosition(Constants.Climb.CLIMB_DOWN_CAGE_POSITION);
    }

    public BooleanSupplier isAtPosition(Position position)
    {
        return () -> Math.abs(leadMotor.getPosition() - position.value) < POSITION_TOLERANCE;
    }

    /**
     * Stops the climb motor
     */
    public void stop()
    {
        set(0.0);
    }
    /**
     * Moves the climb so the robot will climb up the cage
     */
    public Command climbUpCommand()
    {
        return runOnce(() -> climbUpCage()).withName("Climb Up");
    }

    /**
     * Moves the climb so the robot will climb down the cage
     */
    public Command climbDownCommand()
    {
        return runOnce(() -> climbDownCage()).withName("Climb Down");
    }

    /**
     * Moves the climb to the position where the robot will be hanging from the cage
     */
    public Command climbToUpPositionCommand()
    {
        return run(() -> climbToUpPosition()).withName("Move To Up Position");
    }

    /**
     * Moves the climb to the position where the robot will be ready to climb
     */
    public Command climbToDownPositionCommand()
    {
        return run(() -> climbToDownPosition()).withName("Move To Down Position");
    }

    /**
     * Stops the climb motor
     */
    public Command stopCommand()
    {
        return runOnce(() -> stop()).withName("Stops Climb");
    }
    
    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    @Override
    public String toString()
    {
        return "Climb position = " + leadMotor.getPosition();
    }
}
