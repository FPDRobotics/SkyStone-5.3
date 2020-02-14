package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "tele new")
public class TeleopNew extends LinearRobot {

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        waitForStart();

        while(!isStopRequested()) {

            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.a) {
                lowerTRexArm();
            } else {
                raiseTRexArm();
            }
            verticalSlideMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger * .7);

            if (gamepad1.dpad_left) {
                leftIntake.setPower(.5);
                rightIntake.setPower(.5);
            } if (gamepad1.dpad_right) {
                leftIntake.setPower(-.5);
                rightIntake.setPower(-.5);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }

        }

    }

}
