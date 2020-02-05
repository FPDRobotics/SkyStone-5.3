package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous

public class ColorSensorTest extends LinearOpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor TRex;
    private ColorSensor ColorSensor;

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

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        TRex = hardwareMap.dcMotor.get("TRex");
        ColorSensor = hardwareMap.colorSensor.get("ColorSensor");


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

        int CurrentColor;

        waitForStart();
        if (opModeIsActive()) {
            ColorSensor.enableLed(false);
            sleep(2000);
            ColorSensor.enableLed(true);
            while (opModeIsActive()) {
                CurrentColor = Color.rgb(ColorSensor.red(), ColorSensor.green(), ColorSensor.blue());
                telemetry.addData("Hue", JavaUtil.colorToHue(CurrentColor));
                telemetry.addData("Saturation", JavaUtil.colorToSaturation(CurrentColor));
                telemetry.addData("Value", JavaUtil.colorToValue(CurrentColor));
                telemetry.update();
            }

        }
    }


}
