package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.IndexServoAction;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesBlue;

@Config
@Autonomous(name = "Blue Autonomous Far Side [No Webcam] [NO SYSTEMS]", group = "Autonomous")
public class BlueAutoLongNoWebcamNoSystem extends LinearOpMode  {
    @Override
    public void runOpMode() {
        // IMPORT POSITIONS
        RobotFieldPosesBlue Field = new RobotFieldPosesBlue();
        // IMPORT CATAPULT ACTIONS
        CatapultAction CatapultSystem = new CatapultAction(hardwareMap);
        // IMPORT INDEXER ACTIONS
        IndexServoAction IndexerSystem = new IndexServoAction(hardwareMap);
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
        Actions.runBlocking(new SleepAction(0.5));

        // Drive to First Artifact Set
        TrajectoryActionBuilder shootPoseToFirstSet = tabStartPoseToShootPose.endTrajectory().fresh()
                .splineTo(Field.GPPStart.position, Field.GPPStart.heading);
        Actions.runBlocking(shootPoseToFirstSet.build());

        // Pick Up Artifacts and Go Back [First Set]
        TrajectoryActionBuilder pickTab1 = shootPoseToFirstSet.endTrajectory().fresh()
                .lineToX(Field.ArtifactSetEnd_X);
        TrajectoryActionBuilder goBack1 = pickTab1.endTrajectory().fresh()
                .splineTo(Field.GPPStart.position, Field.directionUP);
        Actions.runBlocking(pickTab1.build());
        Actions.runBlocking(goBack1.build());

        // Drive Back to Shoot Pose
        TrajectoryActionBuilder firstSetToShootPose = goBack1.endTrajectory().fresh()
                .splineTo(Field.shootPoseLong.position, Field.shootPoseLong.heading);
        Actions.runBlocking(firstSetToShootPose.build());

        // SHOOT
        Actions.runBlocking(new SleepAction(0.5));

        // Drive to Second Artifact Set
        TrajectoryActionBuilder shootPoseToSecondSet = tabStartPoseToShootPose.endTrajectory().fresh()
                .splineTo(Field.PGPStart.position, Field.PGPStart.heading);
        Actions.runBlocking(shootPoseToSecondSet.build());

        // Pick Up Artifacts and Go Back [Second Set]
        TrajectoryActionBuilder pickTab2 = shootPoseToSecondSet.endTrajectory().fresh()
                .lineToX(Field.ArtifactSetEnd_X);
        TrajectoryActionBuilder goBack2 = pickTab1.endTrajectory().fresh()
                .splineTo(Field.PGPStart.position, Field.directionUP);
        Actions.runBlocking(pickTab2.build());
        Actions.runBlocking(goBack2.build());

        // Drive Back to Shoot Pose
        TrajectoryActionBuilder secondSetToShootPose = goBack2.endTrajectory().fresh()
                .splineTo(Field.shootPoseLong.position, Field.shootPoseLong.heading);
        Actions.runBlocking(secondSetToShootPose.build());

        // SHOOT
        Actions.runBlocking(new SleepAction(0.5));

        // Drive to End Position
        Actions.runBlocking(secondSetToShootPose.endTrajectory().fresh().lineToX(Field.ArtifactSetEnd_X).build());

    }
}
