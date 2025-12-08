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

@Config
@Autonomous(name = "BLUE AUTO [JUST FORWARD]", group = "Autonomous")
public class BlueJustMoveForward extends LinearOpMode  {
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
        Pose2d initialPose = Field.initialFarPose;
        // Initialize Drive
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // GO TO STARTING POSE
        TrajectoryActionBuilder tabStartPoseToStartPose = drive.actionBuilder(initialPose)
                .lineToY(Field.GPPStart.position.y);
        Actions.runBlocking(
                tabStartPoseToStartPose.build()
        );
    }
}
