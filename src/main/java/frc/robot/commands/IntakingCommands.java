package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
// import frc.robot.Constants.TargetPosition;
import frc.robot.subsystems.IntakeWrist.Position;
import frc.robot.subsystems.LEDs.ColorPattern;

public final class IntakingCommands
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
    //BTYG



    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private static Intake intake;
    private static IntakeWrist intakeWrist;
    // private static Pivot pivot;
    private static Elevator elevator;
    private static Claw claw;
    // private static LEDs leds;
    private static Proximity intakeProximity;
    private static Proximity elevatorProximity;
    private static Proximity clawProximity;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here
    private IntakingCommands()
    {}

    public static void createCommands(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        intake = robotContainer.getIntake();
        intakeWrist = robotContainer.getIntakeWrist();
        // pivot = robotContainer.getPivot();
        elevator = robotContainer.getElevator();
        claw = robotContainer.getClaw();
        // leds = robotContainer.getLEDs();
        intakeProximity = robotContainer.getIntakeProximity();
        clawProximity = robotContainer.getShooterProximity();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here BTYG


    /**
     * Command to pickup a coral from the ground
     * @return the command to pickup coral
     * @author Logan Bellinger
     * 
     * NO LONGER BEING USED W/ NEW NO GROUND INTAKE DESIGN
    */
    public static Command intakeCoralCommand()
    {
        if(intake != null && intakeWrist != null && elevator != null && claw != null && intakeProximity != null && elevatorProximity != null && clawProximity != null)
        {
            // Does it work?  I don't know.  I'm sure its fine
            return 
                intake.pickupCoralCommand()
                    .until(intakeProximity.isDetectedSupplier())
                    .withTimeout(3.0)

                    .andThen(Commands.waitSeconds(0.5))

                    .andThen(claw.stopCommand())    
                    
            .andThen(intake.stopCommand())

            .andThen(
                intakeWrist.moveToSetPositionCommand(Position.kRestingPosition)
                    .until(intakeWrist.isAtPosition(Position.kRestingPosition))
                    .withTimeout(2.0))

            .andThen(
                intake.ejectCoralCommand()
                    .until(elevatorProximity.isDetectedSupplier())
                    .withTimeout(2.0))
            
            .andThen(intake.stopCommand())
            // .andThen(Commands.waitSeconds(0.5))
            .andThen(
                Commands.parallel(
                    claw.intakeCoralCommand()
                        .until(clawProximity.isDetectedSupplier()),

                    elevator.moveToSetPositionCommand(ElevatorPosition.kIntakingPosition)
                        .until(elevator.isAtPosition(ElevatorPosition.kIntakingPosition))))

            .andThen(Commands.waitSeconds(0.5))
            .andThen(claw.stopCommand())
            .andThen(GeneralCommands.setLedCommand(ColorPattern.kSolid, Color.kRed));
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command moveIntakeCommand()
    {
        if(elevator != null && intakeWrist != null)
        {
            return
        Commands.parallel(
                intakeWrist.moveToSetPositionCommand(Position.kIntakeCoralPosition)
                    .until(intakeWrist.isAtPosition(Position.kIntakeCoralPosition)),

                GeneralCommands.moveScorerToIntakingPositionCommand());
            // .andThen(
            //     intakeWrist.moveToSetPositionCommand(Position.kRestingPosition)
            //         .until(intakeWrist.isAtPosition(Position.kRestingPosition))
            
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to intake a Coral from the Coral Station
     * @return the command to intake coral
     * @author Logan Bellinger
     */
    public static Command intakeCoralFromStationCommand()
    {
        if(elevator != null && claw != null && clawProximity != null)
        {
            return
            // GeneralCommands.moveScorerToIntakingPositionCommand()
            //     .until(() -> (elevator.isAtPosition(ElevatorPosition.kReadyToGrabCoralPosition).getAsBoolean() && pivot.isAtPosition(PivotPosition.kDownPosition).getAsBoolean()))
            claw.moveSticktoSetPositionCommand(0.0)
                .until(claw.isAtPosition(0.0))

            .andThen(
                Commands.parallel(
                    GeneralCommands.setLedCommand(ColorPattern.kSolid, Color.kYellow).withTimeout(0.25),

                    elevator.moveToSetPositionCommand(ElevatorPosition.kIntakingPosition)
                        .until(elevator.isAtPosition(ElevatorPosition.kIntakingPosition))
                        .withTimeout(2.0),

                    claw.intakeCoralCommand().until(clawProximity.isDetectedSupplier()).andThen(claw.stopCommand())
                ))

            // .andThen(elevator.moveToSetPositionCommand(ElevatorPosition.kSafeSwingPosition).until(elevator.isAtPosition(ElevatorPosition.kSafeSwingPosition)))

            
            // .andThen(Commands.waitSeconds(0.25))

            // .andThen(
            //     Commands.parallel(
            //         claw.grabGamePieceCommand()
            //             .until(clawProximity.isDetectedSupplier())
            //             .withTimeout(1.0),

            //         elevator.moveToSetPositionCommand(ElevatorPosition.kGrabCoralPosition)
            //             .until(elevator.isAtPosition(ElevatorPosition.kGrabCoralPosition))))

            // .andThen(Commands.waitSeconds(0.5))
            // .andThen(claw.stopCommand())
            .andThen(GeneralCommands.setLedCommand(ColorPattern.kSolid, Color.kRed));
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command testCommand1()
    {
        return
        intakeWrist.moveToSetPositionCommand(Position.kIntakeCoralPosition);//.alongWith(intake.pickupCoralCommand());
    }
    public static Command testCommand2()
    {
        return
        intakeWrist.moveToSetPositionCommand(Position.kManipAlgaePosition).alongWith(intake.pickupAlgaeCommand());
    }
    public static Command testCommand3()
    {
        return
        intakeWrist.moveToSetPositionCommand(Position.kRestingPosition).alongWith(intake.stopCommand());
    }
    public static Command testCommand4()
    {
        return
        (intake.ejectAlgaeCommand());
    }

    /**
     * The command to intake algae from the ground with our ground intake
     * @return Command to intake algae
     * @author Logan Bellinger
     */
    public static Command intakeAlgaeCommand()
    {
        if(intake != null && intakeWrist != null && intakeProximity != null)
        {
            return
            Commands.parallel(
                GeneralCommands.setLedCommand(ColorPattern.kBlink, Color.kYellow)
                    .withTimeout(0.1), 

                intakeWrist.moveToSetPositionCommand(Position.kIntakeCoralPosition)
                    .until(intakeWrist.isAtPosition(Position.kIntakeCoralPosition))

            .andThen(
                intake.pickupAlgaeCommand()
                    .until(intake.isAlgaeIn()))

            .andThen(Commands.waitSeconds(0.5))

            .andThen(Commands.parallel(
                intake.pulseCommand(),

                intakeWrist.moveToSetPositionCommand(Position.kManipAlgaePosition)
                    .until(intakeWrist.isAtPosition(Position.kManipAlgaePosition)))
            ));
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command betterStationIntakeCommand()
    {
        if(intakeWrist != null && elevator != null)
        {
            return
            Commands.parallel(
                intakeWrist.moveToSetPositionCommand(Position.kManipAlgaePosition)
                    .until(intakeWrist.isAtPosition(Position.kManipAlgaePosition))
                    .withTimeout(2.0),

                intakeCoralFromStationCommand());
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to intake an Algae from the reef
     * @param targetPosition the position to move the elevator and pivot to, either the UPPER_REEF_ALGAE (between L3 and L4) or the LOWER_REEF_ALGAE (between L2 and L3)
     * @return The command to intake the algae
     * @author Logan Bellinger
     */
    public static Command intakeAlgaeFromReefCommand(ElevatorPosition elevatorPosition)
    {
        if(elevator != null)
        {
            return
            GeneralCommands.setLedCommand(ColorPattern.kBlink, Color.kYellow)

            .andThen(elevator.moveToSetPositionCommand(elevatorPosition))
            
            .andThen(
                claw.intakeCoralCommand()
                    .until(clawProximity.isDetectedSupplier()))
            
            // .andThen(claw.pulseCommand())
            .andThen(GeneralCommands.setLedCommand(ColorPattern.kSolid, Color.kRed))
            .withName("Intake Algae From Reef");
        }
        else
        {
            return Commands.none();
        }
    }


    // public static Command exampleCommand()
    // {
    //     if(subsystems != null)
    //     {
    //         return someCompoundCommand;
    //     }
    //     else
    //         return Commands.none();
    // }
}
