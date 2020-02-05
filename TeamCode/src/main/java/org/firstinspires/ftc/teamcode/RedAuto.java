package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RedAuto extends LinearOpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private Servo Plattform;

    public void ComputeMotorPowers (double vx, double vy, double a, long hold) {
        final double r = 2;
        final double lx = 7.25;
        final double ly = 6.375;

        double w1 = (1 / r) * (vx - vy - (lx + ly) * a);
        double w2 = (1 / r) * (vx + vy + (lx + ly) * a);
        double w3 = (1 / r) * (vx + vy - (lx + ly) * a);
        double w4 = (1 / r) * (vx - vy + (lx + ly) * a);

        FL.setPower(w1);
        FR.setPower(w2);
        BL.setPower(w3);
        BR.setPower(w4);
        sleep(hold);
    }
    @Override
    public void runOpMode() {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        Plattform = hardwareMap.servo.get("Plattform");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Plattform.setPosition(.8);
        waitForStart();
        if (opModeIsActive()) {
            ComputeMotorPowers(0,.5,0,1200);
            ComputeMotorPowers(0,0,0,500);
            ComputeMotorPowers(-.5,0,0,2000);
            ComputeMotorPowers(0,0,0,500);
            Plattform.setPosition(0);
            sleep(750);
            ComputeMotorPowers(.5,0,0,2000);
            Plattform.setPosition(1);
            sleep(500);
            ComputeMotorPowers(0,-.5,0,2750);
            ComputeMotorPowers(0,0,0,250);
            ComputeMotorPowers(-.5,0,0,3000);
            ComputeMotorPowers(0,.5,0,2000);
            ComputeMotorPowers(.5,0,0,1500);
            ComputeMotorPowers(0,0,0,250);
            ComputeMotorPowers(0,-.5,0,2000);
            ComputeMotorPowers(.5,0,0,1000);
            ComputeMotorPowers(0,-.5,0,1500);

            while (opModeIsActive()) {
                telemetry.addData("FL Power", FL.getPower());
                telemetry.addData("FR Power", FR.getPower());
                telemetry.addData("BL Power", BL.getPower());
                telemetry.addData("BR Power", BR.getPower());
                telemetry.update();
            }
        }
    }
}
