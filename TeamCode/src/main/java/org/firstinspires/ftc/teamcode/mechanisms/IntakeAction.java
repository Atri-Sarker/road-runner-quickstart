package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeAction {

    private CRServo intakeMotorBottom;
    private CRServo intakeMotorTop;

    private CRServo finalServo;

    private double pullPower = 1;
    private double pushPower = -1;

    private double lastServoReduction = 0.3;
    private double pushReduction = 0.3;

    private CRServo indexServo;
    public IntakeAction(HardwareMap hwmap) {
        intakeMotorBottom = hwmap.get(CRServo.class, "bottom");
        intakeMotorTop = hwmap.get(CRServo.class, "top");
        intakeMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorTop.setDirection(DcMotorSimple.Direction.FORWARD);
        finalServo = hwmap.get(CRServo.class, "final");
        finalServo.setDirection(DcMotorSimple.Direction.REVERSE);
        indexServo = hwmap.get(CRServo.class, "indexServo");
        indexServo.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public class AbsorbArtifacts implements Action {
        public double duration = 0;
        public double indexPower = 0.3;
        public ElapsedTime indexTime = new ElapsedTime();

        public void resetTime() {
            indexTime.reset();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // RUN INTAKES
            intakeMotorBottom.setPower(pullPower);
            intakeMotorTop.setPower(pullPower);
            finalServo.setPower(pullPower * lastServoReduction);

            // MANAGE INDEXER DIRECTION
            if ( Math.round(indexTime.seconds() * 3) % 2 == 0 ) {
                indexServo.setPower(indexPower);
            } else {
                indexServo.setPower(-1 * indexPower);
            }

            // EXIT CONDITION
            if ( indexTime.seconds() > duration ) {
                intakeMotorBottom.setPower(0);
                intakeMotorTop.setPower(0);
                finalServo.setPower(0);
                indexServo.setPower(0);

                return false;
            }
            return true;
        }
    }

    public Action intakeForSeconds(double seconds) {
        AbsorbArtifacts temp = new AbsorbArtifacts();
        temp.duration = seconds;
        temp.resetTime();
        return temp;
    }

}
