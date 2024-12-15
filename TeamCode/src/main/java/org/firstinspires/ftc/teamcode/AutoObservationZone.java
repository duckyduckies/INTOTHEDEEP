package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutoObservationZone", group="Linear Opmode")
public class AutoObservationZone extends LinearOpMode {
    boolean debug_mode = true;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    PIDController           pidRotate, pidDrive;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    public void move(double x, double y, double rx, double powerScale) {
        x = x * 1.1;
        double denominator,frontLeftPower,backLeftPower,frontRightPower,backRightPower;
        double frontLeftPower_mod,backLeftPower_mod,frontRightPower_mod,backRightPower_mod;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        // modify power by using DcMotorPowerModifierAdv()
        int eqVer = 1;
        frontLeftPower_mod = DcMotorPowerModifierAdv(frontLeftPower, eqVer);
        backLeftPower_mod = DcMotorPowerModifierAdv(backLeftPower, eqVer);
        frontRightPower_mod = DcMotorPowerModifierAdv(frontRightPower, eqVer);
        backRightPower_mod = DcMotorPowerModifierAdv(backRightPower, eqVer);

        frontLeftMotor.setPower(frontLeftPower_mod * powerScale);
        backLeftMotor.setPower(backLeftPower_mod * powerScale);
        frontRightMotor.setPower(frontRightPower_mod * powerScale);
        backRightMotor.setPower(backRightPower_mod * powerScale);

        if (debug_mode) {
            telemetry.addData("frontLeftPower: ", frontLeftPower);
            telemetry.addData("backLeftPower: ", backLeftPower);
            telemetry.addData("frontRightPower: ", frontRightPower);
            telemetry.addData("backRightPower: ", backRightPower);
            telemetry.addData("frontLeftPower_mod: ", frontLeftPower_mod);
            telemetry.addData("backLeftPower_mod: ", backLeftPower_mod);
            telemetry.addData("frontRightPower_mod: ", frontRightPower_mod);
            telemetry.addData("backRightPower_mod: ", backRightPower_mod);
        }
    }


    private void RotateClockwise(double angle, double power) {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (angle+currentAngle); // needs to move this much: diff
        double sign;
        while (Math.abs(diff)>2){ // while there is more than 2 degree to move
            sign = diff/Math.abs(diff);
            // increase the constant to increase turning speed
            turn = sign*power; //turn = sign*0.3; // by Bo
            leftFront =  turn;
            rightFront = -turn;
            leftRear =  turn;
            rightRear =  - turn;

            frontLeftMotor.setPower(leftFront);
            backLeftMotor.setPower(leftRear);
            frontRightMotor.setPower(rightFront);
            backRightMotor.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (angle+currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        frontLeftMotor.setPower(0); //brake
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    private void RotateCounterClockwise(double angle, double power) {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (angle-currentAngle);
        double sign;
        while (Math.abs(diff)>2){
            sign = diff/Math.abs(diff);
            turn = -sign*power; //turn = -1*sign*0.3; // by Bo
            leftFront =  turn;
            rightFront = - turn;
            leftRear =  turn;
            rightRear = - turn;

            frontLeftMotor.setPower(leftFront);
            backLeftMotor.setPower(leftRear);
            frontRightMotor.setPower(rightFront);
            backRightMotor.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90-currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double DcMotorPowerModifier(double Power) {
        return Math.pow(Math.tanh(Power) / Math.tanh(1), 3);
    }

    private double DcMotorPowerModifierAdv(double Power, int eqVer) {
        if (eqVer == 1) {
            return Math.pow(Math.tanh(Power) / Math.tanh(1), 3);
        } else if (eqVer == 2) {
            return Math.pow(Power, 3);
        } else if (eqVer == 3) {
            return Math.pow(Power, 5);
        } else {
            return Power;
        }
    }

    @Override
    public void runOpMode() {


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        /*
        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);
        */

        waitForStart();

        telemetry.addData("Robot", "initializing");
        telemetry.update();

        sleep(1000);

        /*
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        */

        // Initialize the hardware variables
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Set the direction of the motors
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;


        // ****** Action [1] ******

        // Rachel: If following FTC sample code RobotAutoDriveByTime
        move(0.3, -0, 0, 1);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 700)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            telemetry.update();
        }

        /*
        move(0.3 * 1.1, -0, 0, 1);
        try {
            Thread.sleep(700); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        */

        // ****** Action [2] ******

        // Rachel: If following FTC sample code RobotAutoDriveByTime
        move(0, 0, 0, 1);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 200)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            telemetry.update();
        }

        /*
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        try {
            Thread.sleep(200); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        */

        // ****** Action [2] ******

        move(0 * 1.1, 0, -0.3, 1);
        try {
            Thread.sleep(500); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // ****** Action [2] ******

        move(0.3 * 1.1, -0.1, 0, 1);
        try {
            Thread.sleep(500); // Sleep for 0.5 second (500 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        try {
            Thread.sleep(200); // Sleep for 0.2 second (200 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //Insert arm code here
        try {
            Thread.sleep(200); // Sleep for 0.2 second (200 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        move(0 * 1.1, -0, -0.3, 1);
        try {
            Thread.sleep(500); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        move(0 * 1.1, -0.1, -0, 1);

        try {
            Thread.sleep(500); // Sleep for 0.5 second (500 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}