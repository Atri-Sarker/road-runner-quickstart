package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.IndexServoAction;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeAction;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesBlue;

@Config
@Autonomous(name = "Blue Autonomous Far Side [No Webcam]", group = "Autonomous")
public class BlueAutoLongNoWebcam extends LinearOpMode  {
    @Override
    public void runOpMode() {
        // IMPORT POSITIONS
        RobotFieldPosesBlue Field = new RobotFieldPosesBlue();
        // IMPORT CATAPULT ACTIONS
        CatapultAction CatapultSystem = new CatapultAction(hardwareMap);
        // IMPORT INTAKE ACTIONS
        IntakeAction IntakeSystem = new IntakeAction(hardwareMap);
        // Start Position
        Pose2d initialPose = Field.startPoseLong;
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // SCAN WEBCAM [NO]
        Actions.runBlocking(new SleepAction(0.5));

        // Drive to Shooting Position
        TrajectoryActionBuilder tabStartPoseToShootPose = drive.actionBuilder(initialPose).
                splineTo(Field.shootPoseLong.position, Field.shootPoseLong.heading);
        Actions.runBlocking(
                tabStartPoseToShootPose.build()
        );

        // SHOOT
        Actions.runBlocking(CatapultSystem.LeftRightLeft());

        // Drive to First Artifact Set
        TrajectoryActionBuilder shootPoseToFirstSet = tabStartPoseToShootPose.endTrajectory().fresh()
                .splineTo(Field.GPPStart.position, Field.GPPStart.heading);
        Actions.runBlocking(shootPoseToFirstSet.build());

        // Pick Up Artifacts and Go Back [First Set]
        TrajectoryActionBuilder pickTab1 = shootPoseToFirstSet.endTrajectory().fresh()
                .lineToX(Field.ArtifactSetEnd_X);
        TrajectoryActionBuilder goBack1 = pickTab1.endTrajectory().fresh()
                .splineTo(Field.GPPStart.position, Field.directionUP);
        // Intake Action
        Action intakeIndex = IntakeSystem.intakeForSeconds(2);

        Actions.runBlocking(new ParallelAction(intakeIndex, new SequentialAction(pickTab1.build(), goBack1.build())));

        // Drive Back to Shoot Pose
        TrajectoryActionBuilder firstSetToShootPose = goBack1.endTrajectory().fresh()
                .splineTo(Field.shootPoseLong.position, Field.shootPoseLong.heading);
        Actions.runBlocking(firstSetToShootPose.build());

        // SHOOT
        Actions.runBlocking(CatapultSystem.LeftRightLeft());

        // Drive to Second Artifact Set
        TrajectoryActionBuilder shootPoseToSecondSet = tabStartPoseToShootPose.endTrajectory().fresh()
                .splineTo(Field.PGPStart.position, Field.PGPStart.heading);
        Actions.runBlocking(shootPoseToSecondSet.build());

        // Pick Up Artifacts and Go Back [Second Set]
        TrajectoryActionBuilder pickTab2 = shootPoseToSecondSet.endTrajectory().fresh()
                .lineToX(Field.ArtifactSetEnd_X);
        TrajectoryActionBuilder goBack2 = pickTab2.endTrajectory().fresh()
                .splineTo(Field.PGPStart.position, Field.directionUP);

        Actions.runBlocking(new ParallelAction(intakeIndex, new SequentialAction(pickTab2.build(), goBack2.build())));

        // Drive Back to Shoot Pose
        TrajectoryActionBuilder secondSetToShootPose = goBack2.endTrajectory().fresh()
                .splineTo(Field.shootPoseLong.position, Field.shootPoseLong.heading);
        Actions.runBlocking(secondSetToShootPose.build());

        // SHOOT
        Actions.runBlocking(CatapultSystem.LeftRightLeft());

        // Drive to End Position
        Actions.runBlocking(secondSetToShootPose.endTrajectory().fresh().lineToX(Field.ArtifactSetEnd_X).build());

    }
}
