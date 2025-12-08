package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "BLUE AUTO [SCAN CAM] [SHOOT ONCE] [AGAINST WALL]", group = "Autonomous")
public class BlueAutoShootOnceAndBack extends LinearOpMode  {
    @Override
    public void runOpMode() {
        // IMPORT POSITIONS
        RobotFieldPosesBlue Field = new RobotFieldPosesBlue();
        // IMPORT CATAPULT ACTIONS
        CatapultAction CatapultSystem = new CatapultAction(hardwareMap);
        // IMPORT WEBCAM SYSTEM
        // WebcamControl CameraScanner = new WebcamControl(hardwareMap);
        // IMPORT INTAKE ACTIONS
        IntakeAction IntakeSystem = new IntakeAction(hardwareMap);

        // Start facing the goal
        Pose2d initialPose = Field.initialGoalPose;
        // Initialize Drive
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // GO TO STARTING POSE
        TrajectoryActionBuilder tabStartPoseToStartPose = drive.actionBuilder(initialPose)
                .lineToX(Field.startPoseClose.position.x)
                .turnTo(Field.directionUP);
        Actions.runBlocking(
                tabStartPoseToStartPose.build()
        );

        // SCAN WEBCAM
        String scannedMotif = "PGP";

        telemetry.addData("TAG", scannedMotif);
        telemetry.update();
        // DRIVE TO SHOOTING POSITION
        TrajectoryActionBuilder tabStartPoseToShootPose = tabStartPoseToStartPose.endTrajectory().fresh()
                .turnTo(Field.directionFacingGoal)
                .lineToX(Field.shootPoseClose.position.x);
        Actions.runBlocking(
                tabStartPoseToShootPose.build()
        );

        // Shoot [First Load is always PPG]
        Actions.runBlocking(CatapultSystem.getShootAction(scannedMotif, "PPG"));

        // Rotate towards UP and go down
        TrajectoryActionBuilder shootPoseToFirstSet = tabStartPoseToShootPose.endTrajectory().fresh()
                .turnTo(Math.toRadians(270))
                .lineToY(Field.PGPStart.position.y)
                .turnTo(Field.directionFacingRamp);
        Actions.runBlocking(shootPoseToFirstSet.build());
    }
}
