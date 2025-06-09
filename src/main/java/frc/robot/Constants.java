package frc.robot;

import java.lang.invoke.MethodHandles;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public static final double MAX_BATTERY_VOLTAGE          = 12.0;
    public static final double END_OF_MATCH_BATTERY_VOLTAGE = 11.5; // This is the estimated voltage at the end of each match, used in subsystems with setVoltage()
    
    public static final String NETWORK_TABLE_NAME = "TeamLHS";
    public static final String ADVANTAGE_SCOPE_TABLE_NAME = "ASTable";

    // These are the names of the CAN bus set on the roboRIO and CANivore
    public static final String CANIVORE = "CANivore";
    public static final String ROBORIO  = "rio";


    public static class Camera
    {
        public static final String CAMERA_1 = "limelight-one"; // Inside robot, on the right
        public static final String CAMERA_2 = "limelight-two";  // Inside robot, on the left

        public static final String BOT_POSE = "botpose_wpiblue";

        public static final String CAMERA_1_BOT_POSE = CAMERA_1 + "/" + BOT_POSE;
        public static final String CAMERA_2_BOT_POSE = CAMERA_2 + "/" + BOT_POSE;
    }

    public static class Climb
    {
        public static final int LEAD_MOTOR_PORT                              = 16;
        public static final int FOLLOW_MOTOR_PORT                            = 15;
        public static final String MOTOR_CAN_BUS                        = CANIVORE;

        public static final double CLIMB_UP_CAGE_POSITION                    = 50.0;   // Check value once we have robot
        public static final double CLIMB_DOWN_CAGE_POSITION                  = 10.0;      // Check value once we have robot
    }

    public static class Controllers
    {
        public static final int DRIVER_CONTROLLER_PORT                  = 0;
        public static final int OPERATOR_CONTROLLER_PORT                = 1;
    }

    //Drivetrain uses the TunerConstants class for its constants
    // public static class Drivetrain
    // {
    //     public static int FRONT_LEFT_DRIVE_PORT                         = 7;
    //     public static int FRONT_LEFT_STEER_PORT                          = 9;
    //     public static int FRONT_LEFT_ENCODER_PORT                       = 8;

    //     public static int FRONT_RIGHT_DRIVE_PORT                        = 10;
    //     public static int FRONT_RIGHT_STEER_PORT                         = 12;
    //     public static int FRONT_RIGHT_ENCODER_PORT                      = 11;

    //     public static int BACK_LEFT_DRIVE_PORT                          = 4;
    //     public static int BACK_LEFT_STEER_PORT                           = 6;
    //     public static int BACK_LEFT_ENCODER_PORT                        = 5;

    //     public static int BACK_RIGHT_DRIVE_PORT                         = 1;
    //     public static int BACK_RIGHT_STEER_PORT                          = 3;
    //     public static int BACK_RIGHT_ENCODER_PORT                       = 2;
    // }

    public static class Elevator
    {
        public static final int LEFT_MOTOR_PORT                         = 30;
        // public static final int RIGHT_MOTOR_PORT                        = 0;
        public static final String LEFT_MOTOR_CAN_BUS                   = ROBORIO;
        // public static final String RIGHT_MOTOR_CAN_BUS                  = CANIVORE;

        // public static final double L4                                   = 100.0;
        // public static final double UPPER_REEF_ALGAE                     = 90.0;
        // public static final double L3                                   = 80.0;
        // public static final double LOWER_REEF_ALGAE                     = 70.0;
        // public static final double L2                                   = 60.0;
        // public static final double L1                                   = 40.0;
        // public static final double GRAB_CORAL_POSITION                  = 20.0;
        // public static final double RESTING_POSITION                     = 0.0;
    }

    public static class Claw
    {
        public static final int SHOOT_MOTOR_PORT                         = 32;
        public static final int INDEX_MOTOR_PORT                         = 14;
        public static final int STICK_MOTOR_PORT                         = 52;
        public static final String KICK_MOTOR_CAN_BUS                   = ROBORIO;
        public static final String GRAB_MOTOR_CAN_BUS                   = ROBORIO;
    }

    public static class Intake
    {
        public static final int MOTOR_PORT                              = 4;
        // public static final int BOTTOM_MOTOR_PORT                       = 0;
        public static final String MOTOR_CAN_BUS                        = ROBORIO;
        // public static final String BOTTOM_MOTOR_CAN_BUS                 = CANIVORE;
    }

    public static class IntakeWrist
    {
        public static final int MOTOR_PORT                              = 13;
        public static final String MOTOR_CAN_BUS                        = CANIVORE;
    }

    public static class LEDs
    {
        public static final int LED_PORT                                = 0;
        public static final int LED_LENGTH                              = 200;
    }

    public static class Pivot
    {
        // public static final int LEFT_MOTOR_PORT                         = 0;
        public static final int MOTOR_PORT                              = 31;
        public static final String MOTOR_CAN_BUS                        = ROBORIO;

    //     public static final double L4                                   = 40.0;
    //     public static final double L3                                   = 30.0;
    //     public static final double GRAB_REEF_ALGAE                      = 25.0;
    //     public static final double L2                                   = 20.0;
    //     public static final double L1                                   = 10.0;
    //     public static final double HOLD_ALGAE_POSITION                  = 5.0;
    //     public static final double RESTING_POSITION                     = 0.0;
    }

    public static class Proximity
    {
        public static final int CORAL_INTAKE_PORT                       = 3;
        public static final int SHOOTER_PORT                            = 2; // PREVIOUSLY CLAW PORT, 2
        public static final int BACKUP_SHOOTER_PORT                     = 4;
    }

    public static class Shuttle
    {
        public static final int MOTOR_PORT                              = 0;
        public static final String MOTOR_CAN_BUS                        = ROBORIO;
    }

    public static class Gyro
    {
        public static final double BLUE_LEFT_YAW = 180.0;
        public static final double BLUE_MIDDLE_YAW = 90.0;
        public static final double BLUE_RIGHT_YAW = 0.0;
        public static final double RED_LEFT_YAW = 0.0;
        public static final double RED_MIDDLE_YAW = -90.0;
        public static final double RED_RIGHT_YAW = 180.0;
    }

    // public enum TargetPosition
    // {
    //     kStartingPosition(Constants.Elevator.RESTING_POSITION, Constants.Pivot.RESTING_POSITION),
    //     kGrabCoralPosition(Constants.Elevator.GRAB_CORAL_POSITION, Constants.Pivot.RESTING_POSITION),
    //     kHoldAlgaePosition(Constants.Elevator.L1, Constants.Pivot.HOLD_ALGAE_POSITION),
    //     kL1(Constants.Elevator.L1, Constants.Pivot.L1),
    //     kL2(Constants.Elevator.L2, Constants.Pivot.L2),
    //     kLowerReefAlgae(Constants.Elevator.LOWER_REEF_ALGAE, Constants.Pivot.GRAB_REEF_ALGAE),
    //     kL3(Constants.Elevator.L3, Constants.Pivot.L3),
    //     kUpperReefAlgae(Constants.Elevator.UPPER_REEF_ALGAE, Constants.Pivot.GRAB_REEF_ALGAE),
    //     kL4(Constants.Elevator.L4, Constants.Pivot.L4),
    //     kOverride(-4237, -4237);

    //     public final double pivot;
    //     public final double elevator;

    //     private TargetPosition(double elevator, double pivot)
    //     {
    //         this.pivot = pivot;
    //         this.elevator = elevator;
    //     }

    // }
}
