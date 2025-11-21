package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeControl {
    private CRServo intakeMotorBottom;
    private CRServo intakeMotorTop;

    public void init(HardwareMap hwmap) {
        intakeMotorBottom = hwmap.get(CRServo.class, "bottom");
        intakeMotorTop = hwmap.get(CRServo.class, "top");
        intakeMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorTop.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void drive(double intakePower) {
        intakeMotorBottom.setPower(intakePower);
        intakeMotorTop.setPower(intakePower);
    }
}