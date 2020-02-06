package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "SetPositionAuto", group = "Red Auto")

public class NewSetPositionAuto extends LinearOpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor TRex;

    public void Revolution  (double LR, double FB) {
        if (FB >= 2 || FB <= -2) {
            int a1 = (int)(Math.abs(538) * Math.abs(LR) * FB);
            int a2 = (int)(Math.abs(-538) * Math.abs(LR) * FB);

            FL.setTargetPosition(a1);
            FR.setTargetPosition(a2);
            BL.setTargetPosition(a2);
            BR.setTargetPosition(a1);
        } else if (LR >= 2 || LR <= -2) {
            int a1 = (int)(538 * LR * Math.abs(FB));
            int a2 = (int)(-538 * LR * Math.abs(FB));

            FL.setTargetPosition(a1);
            FR.setTargetPosition(a2);
            BL.setTargetPosition(a2);
            BR.setTargetPosition(a1);
        }


        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(0.5);
        FR.setPower(0.5);
        BL.setPower(0.5);
        BR.setPower(0.5);
    }



    @Override
    public void runOpMode() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        Revolution(2,1 );
    }
}
