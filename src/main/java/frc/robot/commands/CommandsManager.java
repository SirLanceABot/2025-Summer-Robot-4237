// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.PoseEstimator;

/** 
 * An example command that uses an example subsystem. 
 */
public final class CommandsManager
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS AND INSTANCE VARIABLES ***
    // private final ExampleSubsystem exampleSubsystem;

    public enum TargetPosition
    {
        kScoreBargePosition(PivotPosition.kScoreBargePosition, ElevatorPosition.kL4),
        kL4(PivotPosition.kL4, ElevatorPosition.kL4),
        // kUpperReefAlgae(PivotPosition.kReefAlgaePosition, ElevatorPosition.kUpperReefAlgae),
        kL3(PivotPosition.kLowLevelCoralPosition, ElevatorPosition.kL3),
        // kLowerReefAlgae(PivotPosition.kReefAlgaePosition, ElevatorPosition.kLowerReefAlgae),
        kL2(PivotPosition.kLowLevelCoralPosition, ElevatorPosition.kL2),
        kL1(PivotPosition.kLowLevelCoralPosition, ElevatorPosition.kL1),
        // kScoreProcessorWithClawPosition(PivotPosition.kScoreProcessorPosition, ElevatorPosition.kScoreProcessorPosition),
        kHoldAlgaePosition(PivotPosition.kHoldAlgaePosition, ElevatorPosition.kIntakingPosition),
        // kReadyToGrabCoralPosition(PivotPosition.kDownPosition, ElevatorPosition.kReadyToGrabCoralPosition),
        // kGrabCoralPosition(PivotPosition.kDownPosition, ElevatorPosition.kGrabCoralPosition),
        kRestingPosition(PivotPosition.kFlippedPosition, ElevatorPosition.kIntakingPosition);

        public final PivotPosition pivot;
        public final ElevatorPosition elevator;

        private TargetPosition(PivotPosition pivot, ElevatorPosition elevator)
        {
            this.pivot = pivot;
            this.elevator = elevator;
        }
    }

    private static Drivetrain drivetrain;
    private static PoseEstimator poseEstimator;

    /**
     * Creates a new ExampleCommand.
     */
    private CommandsManager() 
    {}

    public static void createCommands(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        drivetrain = robotContainer.getDrivetrain();
        poseEstimator = robotContainer.getPoseEstimator();

        GeneralCommands.createCommands(robotContainer);
        IntakingCommands.createCommands(robotContainer);
        ScoringCommands.createCommands(robotContainer);

        createNamedCommands();

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    private static void createNamedCommands()
    {
        // Intaking Commands
        NamedCommands.registerCommand("Intake Coral From Floor", IntakingCommands.intakeCoralCommand());
        NamedCommands.registerCommand("Intake Coral From Station", IntakingCommands.intakeCoralFromStationCommand());
        NamedCommands.registerCommand("Intake Algae From Floor", IntakingCommands.intakeAlgaeCommand());
        // NamedCommands.registerCommand("Intake Upper Level Algae", IntakingCommands.intakeAlgaeFromReefCommand(ElevatorPosition.kUpperReefAlgae));
        // NamedCommands.registerCommand("Intake Lower Level Algae", IntakingCommands.intakeAlgaeFromReefCommand(ElevatorPosition.kLowerReefAlgae));

        // NamedCommands.registerCommand("Flip Scorer", ScoringCommands.flipScorerCommand());

        NamedCommands.registerCommand("Remove Upper Algae", GeneralCommands.deleteUpperAlgaeCommand());
        NamedCommands.registerCommand("Remove Lower Algae", GeneralCommands.deleteLowerAlgaeCommand());
        
        // Score Coral Commands
        NamedCommands.registerCommand("Move Scorer to L4", GeneralCommands.moveScorerToL4Command());
        NamedCommands.registerCommand("Move Scorer to L3", GeneralCommands.moveScorerToL3Command());
        NamedCommands.registerCommand("Move Scorer to L4 with Algae", GeneralCommands.moveScorerToL4WithAlgaeCommand());
        NamedCommands.registerCommand("Move Scorer to Barge", GeneralCommands.moveScorerToBargeCommand());
        // NamedCommands.registerCommand("Move Scorer to Processor", GeneralCommands.moveScorerToProcessorCommand());

        NamedCommands.registerCommand("Finish Scoring Coral", ScoringCommands.finishScoringCoralCommand());
        NamedCommands.registerCommand("Score Coral Only", GeneralCommands.scoreCoralOnlyCommand());
        NamedCommands.registerCommand("Score Coral Prox", GeneralCommands.scoreCoralProxCommand());

        NamedCommands.registerCommand("Score Coral Right", ScoringCommands.scoreCoralAutonomouslyReallyCoolAndAwesomeCommand(() -> drivetrain.getState().Pose, (() -> poseEstimator.closestBranchLocation(() -> poseEstimator.getPrimaryTagID(), true))));
        NamedCommands.registerCommand("Score Coral Left", ScoringCommands.scoreCoralAutonomouslyReallyCoolAndAwesomeCommand(() -> drivetrain.getState().Pose, (() -> poseEstimator.closestBranchLocation(() -> poseEstimator.getPrimaryTagID(), false))));
        // NamedCommands.registerCommand("Score Algae Only", GeneralCommands.scoreAlgaeOnlyCommand());

        // NamedCommands.registerCommand("Auto Align Coral", ScoringCommands.scoreCoralAutonomouslyReallyCoolAndAwesomeCommand)

        // Score Algae Commands
        NamedCommands.registerCommand("Score Algae in Processor", ScoringCommands.scoreProcessorWithIntakeCommand());
        // NamedCommands.registerCommand("Score Algae in Barge", ScoringCommands.scoreAlgaeInBargeCommand());

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }
}
