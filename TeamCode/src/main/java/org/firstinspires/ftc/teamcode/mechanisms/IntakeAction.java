package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeAction {

    private CRServo intakeMotorBottom;
    private CRServo intakeMotorTop;

    private CRServo finalServo;

    private double pullPower = 1;
    private double pushPower = -1;

    private double lastServoReduction = 0.3;
    private double pushReduction = 0.3;

    public void IntakeAction(HardwareMap hwmap) {
        intakeMotorBottom = hwmap.get(CRServo.class, "bottom");
        intakeMotorTop = hwmap.get(CRServo.class, "top");
        intakeMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorTop.setDirection(DcMotorSimple.Direction.FORWARD);
        finalServo = hwmap.get(CRServo.class, "final");
        finalServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class Pull implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotorBottom.setPower(pullPower);
            intakeMotorTop.setPower(pullPower);
            finalServo.setPower(pullPower * lastServoReduction);
            return false;
        }
    }

    public class Push implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotorBottom.setPower(pushPower * pushReduction);
            intakeMotorTop.setPower(pushPower * pushReduction );
            finalServo.setPower(pushPower * pushReduction);
            return false;
        }
    }

    public class Stop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotorBottom.setPower(pushPower * pushReduction);
            intakeMotorTop.setPower(pushPower * pushReduction );
            finalServo.setPower(pushPower * pushReduction);
            return false;
        }
    }

}
