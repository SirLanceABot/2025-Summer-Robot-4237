package frc.robot.tests;

import java.lang.annotation.ElementType;
import java.lang.invoke.MethodHandles;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.commands.CommandsManager.TargetPosition;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Claw;
// import frc.robot.Constants.TargetPosition;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.IntakeWrist.Position;
import frc.robot.subsystems.LEDs.ColorPattern;
import frc.robot.subsystems.Pivot.PivotPosition;

@SuppressWarnings("unused")
public class LoganBTest implements Test
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

    private final Climb climb;
    private final Pivot pivot;
    private final Intake intake;
    private final IntakeWrist intakeWrist;
    private final Claw claw;
    private final Elevator elevator;
    private final PoseEstimator poseEstimator;
    private final Camera climbSideLimelight;
    private final Camera scoringSideLimelight;
    private final Joystick joystick = new Joystick(0);
    // private final ExampleSubsystem exampleSubsystem;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public LoganBTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        // this.exampleSubsystem = robotContainer.exampleSubsystem;
        climb = robotContainer.getClimb();
        pivot = robotContainer.getPivot();
        intake = robotContainer.getIntake();
        intakeWrist = robotContainer.getIntakeWrist();
        claw = robotContainer.getClaw();
        elevator = robotContainer.getElevator();
        poseEstimator = robotContainer.getPoseEstimator();
        climbSideLimelight = robotContainer.getClimbSideCamera();
        scoringSideLimelight = robotContainer.getScoringSideCamera();
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
    {
        
    }

    /**
     * This method runs periodically (every 20ms).
     * BOW TO YOUR GOD
     */
    public void periodic()
    {
        if(joystick.getRawButton(1))
        {
            // IntakingCommands.intakeCoralFromStationCommand().schedule();
            // elevator.moveToSetPositionCommand(ElevatorPosition.kIntakingPosition).schedule();
            // claw.shootCoral();
            // intake.set(0.5);
            // IntakingCommands.intakeCoralCommand().schedule();
            // elevator.moveToSetPositionCommand(ElevatorPosition.kL4).schedule();
            // elevator.set(0.1);
            // intakeWrist.moveToSetPositionCommand(Position.kRestingPosition).schedule();
            // intakeWrist.set(0.2);
            // IntakingCommands.intakeCoralCommand().schedule();
            // GeneralCommands.testgrabbingFromIntakeCommand().schedule();
            // pivot.set(0.1);
            // claw.grabGamePiece();
            // elevator.moveToSetPositionCommand(ElevatorPosition.kSafeSwingPosition).schedule();
            // GeneralCommands.moveScorerToIntakingPositionCommand().schedule();
            // pivot.set(0.1);
            // GeneralCommands.moveScorerToL1Command().schedule();
            // climb.set(0.3);
            // IntakingCommands.intakeAlgaeFromReefCommand(TargetPosition.kLowerReefAlgae).schedule();
            claw.setStickSpeed(0.025);
        }
        else if(joystick.getRawButton(2))
        {
            // GeneralCommands.moveScorerToL4Command().schedule();
            // elevator.moveToSetPositionCommand(ElevatorPosition.kL2).schedule();
            // elevator.set(0.2);
            // intake.set(-0.5);
            // ScoringCommands.flipScorerCommand().schedule();
            // IntakingCommands.intakeAlgaeFromReefCommand(TargetPosition.kUpperReefAlgae).schedule();
            // ScoringCommands.flipScorerCommand().schedule();
            // elevator.moveToSetPositionCommand(ElevatorPosition.kL2).schedule();
            // elevator.set(-0.1);
            // intakeWrist.moveToSetPositionCommand(Position.kIntakeCoralPosition).schedule();
            // intakeWrist.set(-0.2);
            // pivot.moveToSetPositionCommand(PivotPosition.kDownPosition).schedule();
            // pivot.set(-0.1);
            // GeneralCommands.testGrabAndFlipCommand().schedule();
            // pivot.set(0.2);
            // elevator.moveToSetPositionCommand(ElevatorPosition.kL4).schedule();
            // elevator.set(-0.2);
            // pivot.set(-0.1);
            // GeneralCommands.moveScorerToL2Commmand().schedule();
            // climb.set(-0.3);
            claw.setStickSpeed(-0.1);
        }
        else if(joystick.getRawButton(3))
        {
            claw.moveStickToSetPosition(0);
            // GeneralCommands.scoreCoralOnlyCommand().schedule();
            // elevator.moveToSetPositionCommand(ElevatorPosition.kL3).schedule();
            // GeneralCommands.moveScorerToBargeCommand().schedule();
            // pivot.moveToSetPositionCommand(PivotPosition.kFlippedPosition).schedule();
            // elevator.set(-0.2);
            // elevator.set(0.2);
            // pivot.set(0.1);
            // intake.pickupCoral();
            // pivot.set(-0.2);
            // elevator.moveToSetPositionCommand(ElevatorPosition.kReadyToGrabCoralPosition).schedule();
            // GeneralCommands.moveScorerToL3Command().schedule();
        }
        else if(joystick.getRawButton(4))
        {
            claw.moveStickToSetPosition(1.8);
            // elevator.moveToSetPositionCommand(ElevatorPosition.kIntakingPosition).until(elevator.isAtPosition(ElevatorPosition.kIntakingPosition)).withTimeout(1.0).schedule();            // elevator.moveToSetPositionCommand(ElevatorPosition.kL4).schedule();
            // ScoringCommands.testHoldAlgaeCommand().schedule();
            // GeneralCommands.moveScorerToProcessorCommand().schedule();
            // pivot.moveToSetPositionCommand(PivotPosition.kHoldAlgaePosition).schedule();
            // pivot.set(-0.1);
            // intake.pickupAlgae();
            // elevator.set(-0.2);
            // GeneralCommands.moveScorerToL4Command().schedule();
        }
        else if(joystick.getRawButton(5))
        {
            claw.setStickPosition(0);
            // claw.shootCoralCommand().schedule();
            // elevator.moveToSetPositionCommand(ElevatorPosition.kScoreProcessorPosition).schedule();
            // claw.placeCoral();
            // GeneralCommands.moveScorerToIntakingPositionCommand().schedule();
        }
        else if(joystick.getRawButton(6))
        {
            // claw.grabGamePiece();
            // ScoringCommands.flipScorerCommand().schedule();
        }
        else
        {
            // elevator.stopCommand().schedule();
            // pivot.stopCommand().schedule();
            // claw.stopCommand().schedule();
            // pivot.stop();
            // climb.stop();
            // claw.stop();
            // pivot.stop();
            // elevator.stop();
            // intake.stop();
            // claw.stopCommand().schedule();
            claw.stopStick();
        }
        System.out.println("Stick pos: " + claw.getStickPosition());
        // System.out.println("Elevator Pos: " + elevator.getPosition());








        // if(joystick.getRawButton(1)) // A button
        // {
        //     poseEstimator.setPlacingSideToLeftCommand().schedule();
        //     System.out.println("Placing side set to Left");
        // }
        // else if(joystick.getRawButton(2))
        // {
        //     poseEstimator.setPlacingSideToRightCommand().schedule();
        //     System.out.println("Placing side set to Right");
        // }
        // else if(joystick.getRawButton(3))
        // {
        //     // TODO: SET THE CLIMB SIDE LL IN ROBOOTCONTAINER TO TRUE (OR SCORING SIDE IF
        //     // WE ARE USING THE OTHER ONE. MAKE SURE YOU USE THE RIGHT CAMERA OBJECT!!!!
        //     // TODO: DOUBLE CHECK THE LL IS NAMED CORRECTLY
        //     // TODO: DOUBLE CHECK LL IS ON MOST RECENT 2024 FIRMWARE

        //     // Retrieve the current estimated pose and the nearest scoring pose
        //     // Pose2d currentPose = poseEstimator.getEstimatedPose();
        //     Pose2d currentPose = climbSideLimelight.getPose();
        //     int tagId = (int) climbSideLimelight.getTagId();
        //     boolean validTagInFrame = climbSideLimelight.isValidTagInFrame();
        //     int tagCount = climbSideLimelight.getTagCount();

        //     // print out the primary tagId as well as the number of tags from the mt2 pose
        //     System.out.printf("Primary Tag ID: %d | Valid Tag: %b | Tag Count: %d%n",
        //             tagId, validTagInFrame, tagCount);

        //     if(tagCount != 0 && tagId != -1)
        //     {
        //         boolean isRightBranch = poseEstimator.getIsRightBranch();
        //         Pose2d scoringPose = poseEstimator.closestBranchLocation(tagId, isRightBranch);

        //         // Calculate the total distance between the current pose and the scoring pose
        //         Transform2d poseDifference = scoringPose.minus(currentPose);
        //         double distance = climbSideLimelight.avgTagDistance();

        //         // Print all relevant info on one line
        //         System.out.printf("Pose: (X: %.2f, Y: %.2f) | Scoring Node: (X: %.2f, Y: %.2f, Rot: %s) | Distance: %.2f%n",
        //                 currentPose.getX(), currentPose.getY(),
        //                 scoringPose.getX(), scoringPose.getY(), scoringPose.getRotation(),
        //                 distance);
        //     }
        //     else 
        //     {
        //         System.out.println("No tags found");
        //     }
            
        // }
        // System.out.println(climbSideLimelight.getTagId());
        // System.out.println(climbSideLimelight.g);
        // System.out.println(climbSideLimelight.getCameraName());
        // else if(joystick.getRawButton(2)) // B button
        // {
        //     // climb.climbDown();
        //     // pivot.moveToSetPositionCommand(TargetPosition.kL2).schedule(); // value of 1.0 from motor encoder
        //     // intake.ejectCommand().schedule();
        //     // intakeWrist.moveToSetPositionCommand(Position.kShootingPosition).schedule(); // 20.0
            // claw.ejectAlgaeCommand().schedule();
        //     // elevator.moveToSetPositionCommand(TargetPosition.kStartingPosition).schedule(); // 20.0
        // }
        // else if(joystick.getRawButton(3)) // X button
        // {
        //     // pivot.moveToSetPositionCommand(TargetPosition.kL3).schedule();
        // }
        // else if(joystick.getRawButton(4)) // Y button
        // {
        //     // pivot.moveToSetPositionCommand(TargetPosition.kL4).schedule();
        // }

        // System.out.println(intakeWrist.getPosition());
        // System.out.println(intakeWrist.getPosition());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}

