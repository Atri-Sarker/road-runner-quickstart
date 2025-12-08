package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeAction;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesBlue;
import org.firstinspires.ftc.teamcode.mechanisms.RobotFieldPosesRed;
import org.firstinspires.ftc.teamcode.mechanisms.WebcamControl;

@Config
@Autonomous(name = "BLUE AUTO [SCAN CAM] [SHOOT THRICE]", group = "Autonomous")
public class BlueAutoShootThriceWithCam extends LinearOpMode  {
    @Override
    public void runOpMode() {
        // IMPORT POSITIONS
        RobotFieldPosesBlue Field = new RobotFieldPosesBlue();
        // IMPORT CATAPULT ACTIONS
        CatapultAction CatapultSystem = new CatapultAction(hardwareMap);
        // IMPORT WEBCAM SYSTEM
        WebcamControl CameraScanner = new WebcamControl(hardwareMap);
        // IMPORT INTAKE ACTIONS
        IntakeAction IntakeSystem = new IntakeAction(hardwareMap);

        // Start at a square facing the motif
        Pose2d initialPose = Field.startPoseClose;
        // Initialize Drive
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // SCAN WEBCAM
        String scannedMotif = CameraScanner.waitForDecodeMotif(5.0);

        telemetry.addData("TAG", scannedMotif);
        telemetry.update();
        // DRIVE TO SHOOTING POSITION
        TrajectoryActionBuilder tabStartPoseToShootPose = drive.actionBuilder(initialPose)
                .turnTo(Field.directionFacingGoal)
                .lineToX(Field.shootPoseClose.position.x);
        Actions.runBlocking(
                tabStartPoseToShootPose.build()
        );

        // Shoot [First Load is always PPG]
        Actions.runBlocking(CatapultSystem.getShootAction(scannedMotif, "PPG"));

        // Rotate towards UP and go down
        TrajectoryActionBuilder shootPoseToFirstSet = tabStartPoseToShootPose.endTrajectory().fresh()
                .turnTo(Field.directionUP)
                .lineToY(Field.PGPStart.position.y)
                .turnTo(Field.directionFacingRamp);
        Actions.runBlocking(shootPoseToFirstSet.build());

        // Pick Up Artifacts and Go Back [First Set]
        TrajectoryActionBuilder pickTab1 = shootPoseToFirstSet.endTrajectory().fresh()
                .lineToX(Field.ArtifactSetEnd_X);
        TrajectoryActionBuilder goBack1 = pickTab1.endTrajectory().fresh()
                .lineToX(Field.PGPStart.position.x)
                .turnTo(Field.directionUP)
                .lineToY(Field.shootPoseClose.position.y)
                .turnTo(Field.directionFacingGoal);
        // Intake Action
        Action intakeIndex = IntakeSystem.intakeForSeconds(4.0);

        Actions.runBlocking(new ParallelAction(intakeIndex, new SequentialAction(pickTab1.build(), goBack1.build())));

        // Shoot [Second Load is always xxx]
        Actions.runBlocking(CatapultSystem.getShootAction(scannedMotif, "PPG"));

        // Drive Back
        TrajectoryActionBuilder moveOut = goBack1.endTrajectory().fresh()
                .turnTo(Field.directionUP)
                .lineToY(Field.GPPStart.position.y)
                .turnTo(Field.directionFacingRamp);

        Actions.runBlocking(moveOut.build());

        // Pick Up Artifacts and Go Back [First Set]
        TrajectoryActionBuilder pickTab2 = moveOut.endTrajectory().fresh()
                .lineToX(Field.ArtifactSetEnd_X);
        TrajectoryActionBuilder goBack2 = pickTab2.endTrajectory().fresh()
                .lineToX(Field.GPPStart.position.x)
                .turnTo(Field.directionUP)
                .lineToY(Field.shootPoseClose.position.y)
                .turnTo(Field.directionFacingGoal);
        // Intake Action
        Action intakeIndex2 = IntakeSystem.intakeForSeconds(4.0);

        Actions.runBlocking(new ParallelAction(intakeIndex, new SequentialAction(pickTab2.build(), goBack2.build())));

        // Shoot [Third Load is always xxx]
        Actions.runBlocking(CatapultSystem.getShootAction(scannedMotif, "GPP"));

        // Drive Back
        TrajectoryActionBuilder moveOut2 = goBack2.endTrajectory().fresh()
                .turnTo(Field.directionUP)
                .lineToY(Field.GPPStart.position.y)
                .turnTo(Field.directionFacingRamp);

        Actions.runBlocking(moveOut2.build());




    }
}
