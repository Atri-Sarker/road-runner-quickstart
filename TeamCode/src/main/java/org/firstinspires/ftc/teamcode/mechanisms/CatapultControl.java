package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CatapultControl {
    private DcMotor leftCatapult, rightCatapult;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();

    private double CATAPULT_UP_POWER = -1;
    private double CATAPULT_DOWN_POWER = 1;
    private enum CatapultModes {UP, DOWN, BRAKE}
    private CatapultModes pivotMode;

    private String SHOOTING = "FALSE";


    public void init(HardwareMap hwmap) {
        leftCatapult = hwmap.get(DcMotor.class, "left_catapult");
        rightCatapult = hwmap.get(DcMotor.class, "right_catapult");

        rightCatapult.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        leftCatapult.setDirection(DcMotor.Direction.FORWARD);

        rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();
        catatime.reset();
    }

    public void handleCatapult(boolean shoot) {
        if (SHOOTING.equals("FALSE") && shoot) {
            SHOOTING = "START";
        }
        if (SHOOTING.equals("START")) {
            catatime.reset();
            leftCatapult.setPower(CATAPULT_DOWN_POWER);
            rightCatapult.setPower(CATAPULT_DOWN_POWER);
            SHOOTING = "DOWN";
        }
        if (SHOOTING.equals("DOWN")) {
            if (catatime.seconds() > 0.4) {
                SHOOTING = "UP";
                catatime.reset();
                rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        if (SHOOTING.equals("UP")) {
            if (catatime.seconds() > 0.1) {
                leftCatapult.setPower(CATAPULT_UP_POWER);
                rightCatapult.setPower(CATAPULT_UP_POWER);
            }
            if (catatime.seconds() > 0.34) {
                leftCatapult.setPower(0);
                rightCatapult.setPower(0);
                SHOOTING = "FALSE";
            }
        }
    }
}