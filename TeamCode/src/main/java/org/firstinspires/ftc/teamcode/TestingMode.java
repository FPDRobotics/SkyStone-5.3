package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="testing")
public class TestingMode extends LinearRobot {


    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        setZeroPowerDrivetrain(DcMotor.ZeroPowerBehavior.FLOAT);
        trexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        trexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        trexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        trexMotor.setPower(0);

        waitForStart();

        while(!isStopRequested()) {

            telemetry.addData("FL", FLdrive.getCurrentPosition());
            telemetry.addData("FR", FRdrive.getCurrentPosition());
            telemetry.addData("Bl", BLdrive.getCurrentPosition());
            telemetry.addData("BR", BRdrive.getCurrentPosition());

            telemetry.addData("TRex", trexMotor.getCurrentPosition());

            telemetry.addData("Distance sensor", distanceSensor.getDistance(DistanceUnit.INCH));

            telemetry.addData("color SL", colorSL.alpha());
            telemetry.addData("color SR", colorSR.alpha());
            telemetry.addData("argb sl", colorSL.argb());
            telemetry.addData("argb sr" , colorSR.argb());
            telemetry.update();

        }

    }



}

