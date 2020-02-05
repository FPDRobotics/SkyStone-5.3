package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class Control extends LinearOpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor IntakeLeft;
    private DcMotor IntakeRight;
    private DcMotor Lift;
    private DcMotor TRex;
    private Servo Plattform;
    private Servo clamp;



    @Override
    public void runOpMode() {

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        IntakeLeft = hardwareMap.dcMotor.get("IntakeLeft");
        IntakeRight = hardwareMap.dcMotor.get("IntakeRight");
        Lift = hardwareMap.dcMotor.get("Lift");
        TRex = hardwareMap.dcMotor.get("TRex");
        Plattform = hardwareMap.servo.get("Plattform");
        clamp = hardwareMap.servo.get("clamp");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_x);
                double robotAngle = Math.atan2(-gamepad1.right_stick_x, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.left_stick_y;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                BL.setPower(v1 * .7);
                FR.setPower(v2 * .7);
                FL.setPower(v3 * .7);
                BR.setPower(v4 * .7);

                if (gamepad1.dpad_right) {
                    IntakeLeft.setPower(1);
                    IntakeRight.setPower(1);
                } else if (gamepad1.dpad_left) {
                    IntakeLeft.setPower(-.6);
                    IntakeRight.setPower(-.6);
                } else {
                    IntakeLeft.setPower(0);
                    IntakeRight.setPower(0);
                }
                 if (gamepad1.a) {
                    clamp.setPosition(.6);
                } else if (gamepad1.b) {
                    clamp.setPosition(1);
                } if (gamepad1.x) {
                    Plattform.setPosition(.8);
                } else if (gamepad1.y) {
                    Plattform.setPosition(0);
                }

                Lift.setPower((gamepad1.right_trigger - gamepad1.left_trigger) / 2);

            }
        }
    }
}
