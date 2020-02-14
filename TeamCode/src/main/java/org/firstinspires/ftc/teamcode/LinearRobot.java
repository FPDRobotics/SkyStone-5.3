package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class LinearRobot extends LinearOpMode {

    public DcMotor leftIntake, rightIntake;
    public DcMotor FLdrive, FRdrive, BLdrive, BRdrive;
    public DcMotor trexMotor, verticalSlideMotor;
    public Servo plattformServo, camServo;
    public CRServo horizontalSlideServo;
    public ColorSensor colorSR, colorSL;
    public DistanceSensor distanceSensor;
    public BNO055IMU IMU;

    public Orientation initialOrientation;
    public double initialZPositionDegrees;

    private int raisedTRexArmPosition = 0;
    private int loweredTRexArmPosition = -780;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

    }

    private void initialize() {
        leftIntake = hardwareMap.dcMotor.get(HardwareNames.leftIntakeMotor);
        rightIntake = hardwareMap.dcMotor.get(HardwareNames.rightIntakeMotor);
        FLdrive = hardwareMap.dcMotor.get(HardwareNames.frontLeftMotor);
        FRdrive = hardwareMap.dcMotor.get(HardwareNames.frontRightMotor);
        BLdrive = hardwareMap.dcMotor.get(HardwareNames.backLeftMotor);
        BRdrive = hardwareMap.dcMotor.get(HardwareNames.backRightMotor);
        trexMotor = hardwareMap.dcMotor.get(HardwareNames.trexMotor);
        verticalSlideMotor = hardwareMap.dcMotor.get(HardwareNames.verticalSlideMotor);
        plattformServo = hardwareMap.servo.get(HardwareNames.plattformServo);
        camServo = hardwareMap.servo.get(HardwareNames.clampServo);
        distanceSensor = hardwareMap.get(DistanceSensor.class, HardwareNames.distanceSensor);
        colorSL = hardwareMap.get(ColorSensor.class, HardwareNames.colorSensorLeft);
        colorSR = hardwareMap.get(ColorSensor.class, HardwareNames.colorSensorRight);
        horizontalSlideServo = hardwareMap.get(CRServo.class, HardwareNames.horizontalSlideCRServo);

        //add more devices here ...

        //IMU initialization here
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "imu";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        IMU = hardwareMap.get(BNO055IMU.class, HardwareNames.imu);
        IMU.initialize(IMUparameters);

        initialOrientation = IMU.getAngularOrientation().toAxesOrder(AxesOrder.XYZ);
        initialZPositionDegrees = initialOrientation.thirdAngle * (180/Math.PI);

        setZeroPowerDrivetrain(DcMotor.ZeroPowerBehavior.BRAKE);

        setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

        trexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        raiseTRexArm();

        FRdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BRdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double x, double y, double r) {

        double pwr = Math.hypot(x, y);
        double angle = Math.atan2(y, x) - (Math.PI/4);

        double FL = pwr * Math.cos(angle) + r;
        double FR = pwr * Math.sin(angle) - r;
        double BL = pwr * Math.sin(angle) + r;
        double BR = pwr * Math.cos(angle) - r;

        setDrivetrainPowers(FL, FR, BL, BR);

    }

    /**
     * @param distFromWall should be in inches, relative to wall the robot is pointing at
     * @param gyroHeading in degrees, relative to front of robot
     */
    public void driveSensor(double distFromWall, double gyroHeading, double speed) {

        rotate(gyroHeading);

        double deltaDist;
        double heading;

        double headingIdeal = getZPosition();

        do {

            deltaDist = distFromWall - distanceSensor.getDistance(DistanceUnit.INCH);
            heading = (headingIdeal - getZPosition()) * 0.1;

            drive(((deltaDist < 0.0) ? -1.0 : 1.0) * speed * (Math.abs(deltaDist) * 0.1), 0, (Math.abs(heading) > 0.05) ? heading * 1.2 : 0);

            telemetry.addData("dist", deltaDist);
            telemetry.addData("headDiff", heading);
            telemetry.update();

        } while(Math.abs(deltaDist) > 0.1 && opModeIsActive());

        rotate(headingIdeal);


    }

    /**
     * @param heading in degrees relative to the front of the robot
     */
    public void rotate(double heading) {

        double headingToRadians = heading * (Math.PI/180);
        double deltaPos;

        do {
            deltaPos = headingToRadians - getZPosition();

            double rotationMultiplier = 0.5 * Math.abs(deltaPos);

            drive(0,0, ((deltaPos < 0) ? -1.0 : 1.0 ) * rotationMultiplier);

        } while(Math.abs(deltaPos) > 0.023 && opModeIsActive());

        drive(0, 0,0 );
    }

    public void raiseTRexArm() {
        trexMotor.setTargetPosition(raisedTRexArmPosition);
        trexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trexMotor.setPower(0.5);
    }

    public void lowerTRexArm() {
        trexMotor.setTargetPosition(loweredTRexArmPosition);
        trexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trexMotor.setPower(0.5);

    }

    public void moveSkyblockPositions(int positions) {
        int fl = -370,
            fr = -370,
            bl = -370,
            br = -370;

        addTargetPositions(fl * positions, fr * positions, bl * positions, br * positions);
        setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivetrainPowers(0.5, 0.5, 0.5, 0.5);

        while(areDrivetrainMotorsBusy()) {
            idle();
        }

        setDrivetrainPowers(0, 0, 0, 0);
        setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }




    public double getZPosition() { //in radians
        return IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle - initialOrientation.thirdAngle;
    }

    public void setDrivetrainPowers(double FL, double FR, double BL, double BR) {
        FLdrive.setPower(FL);
        FRdrive.setPower(FR);
        BLdrive.setPower(BL);
        BRdrive.setPower(BR);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        FLdrive.setMode(mode);
        FRdrive.setMode(mode);
        BLdrive.setMode(mode);
        BRdrive.setMode(mode);
    }

    public void addTargetPositions(int fl, int fr, int bl, int br) {
        FLdrive.setTargetPosition(fl + FLdrive.getCurrentPosition());
        FRdrive.setTargetPosition(fl + FRdrive.getCurrentPosition());
        BLdrive.setTargetPosition(fl + BLdrive.getCurrentPosition());
        BRdrive.setTargetPosition(fl + BRdrive.getCurrentPosition());
    }

    public void setZeroPowerDrivetrain(DcMotor.ZeroPowerBehavior behavior) {
        FLdrive.setZeroPowerBehavior(behavior);
        FRdrive.setZeroPowerBehavior(behavior);
        BRdrive.setZeroPowerBehavior(behavior);
        BLdrive.setZeroPowerBehavior(behavior);
    }

    public boolean areDrivetrainMotorsBusy() {

        return FLdrive.isBusy() || FRdrive.isBusy() || BLdrive.isBusy() ||  BRdrive.isBusy();

    }

    public void computeMotorPowers (double vx, double vy, double a, long hold) {
        final double r = 2;
        final double lx = 7.25;
        final double ly = 6.375;

        double w1 = (1 / r) * (vx - vy - (lx + ly) * a);
        double w2 = (1 / r) * (vx + vy + (lx + ly) * a);
        double w3 = (1 / r) * (vx + vy - (lx + ly) * a);
        double w4 = (1 / r) * (vx - vy + (lx + ly) * a);

        FLdrive.setPower(w1);
        FRdrive.setPower(w2);
        BLdrive.setPower(w3);
        BRdrive.setPower(w4);
        sleep(hold);
    }

}
