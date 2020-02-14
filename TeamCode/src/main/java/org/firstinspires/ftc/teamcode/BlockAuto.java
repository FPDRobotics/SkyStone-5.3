package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlockAuto extends LinearRobot {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        driveSensor(-28, 0, 0.5);
        moveSkyblockPositions(-1);
        if (colorSL.alpha() <= 100) {
            colorSL.enableLed(false);
            moveSkyblockPositions(-1);
            driveSensor(35, 0, 0.5);
            lowerTRexArm();
            sleep(1000);
            driveSensor(19, 0, 0.5);
            computeMotorPowers(.5, 0, 0, 2000);
            telemetry.addData("hjfdireb", colorSL.alpha());
            telemetry.update();
        }
    }
}
