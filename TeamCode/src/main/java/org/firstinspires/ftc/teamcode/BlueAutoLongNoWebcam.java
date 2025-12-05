package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.CatapultAction;
import org.firstinspires.ftc.teamcode.mechanisms.IndexServoAction;
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
        // IMPORT INDEXER ACTIONS
        IndexServoAction IndexerSystem = new IndexServoAction(hardwareMap);
        // Start Position
        Pose2d initialPose = Field.startPoseLong;
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // SCAN WEBCAM [NO]

        // Drive to Shooting Position

    }
}
