package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndexServoAction {

    private CRServo indexServo;

    public IndexServoAction(HardwareMap hwmap) {
        indexServo = hwmap.get(CRServo.class, "indexServo");
        indexServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void ultraLeft() {
        indexServo.setPower(-0.4);
    }
    public void ultraRight() {
        indexServo.setPower(0.4);
    }

    public void stop() {
        indexServo.setPower(0);
    }

    public void smallLeft() {
        indexServo.setPower(-0.2);
    }
    public void smallRight() {
        indexServo.setPower(0.2);
    }





}
