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
    boolean debugMode = true;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    PIDController           pidRotate, pidDrive;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    double                  rotated;
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

        if (debugMode) {
            /*
            telemetry.addData("frontLeftPower: ", frontLeftPower);
            telemetry.addData("backLeftPower: ", backLeftPower);
            telemetry.addData("frontRightPower: ", frontRightPower);
            telemetry.addData("backRightPower: ", backRightPower);
            telemetry.addData("frontLeftPower_mod: ", frontLeftPower_mod);
            telemetry.addData("backLeftPower_mod: ", backLeftPower_mod);
            telemetry.addData("frontRightPower_mod: ", frontRightPower_mod);
            telemetry.addData("backRightPower_mod: ", backRightPower_mod);
            telemetry.update();
             */
        }
    }

    // angle: + clockwise; - counter-clockwise
    // power: always + here
    // This function only supports moving MORE THAN 2 degree
    private void rotate(int degrees, double power) {
        imu.resetYaw(); // set to 0 degree
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        if (degrees > 0) {
            diff = (degrees + currentAngle); // needs to move this much: diff
        } else {
            diff = (-degrees - currentAngle); // needs to move this much: diff
        }
        double absDiff = Math.abs(diff);
        double sign;
        double power2;

        // Tries to move at least 2 degree to get off 0
        while (opModeIsActive() && Math.abs(diff)>=absDiff-2) {
            sign = diff / Math.abs(diff);
            if (degrees > 0) {
                // increase the power to increase turning speed
                turn = sign * power; //turn = sign*0.3; // by Bo
            } else {
                turn = -sign * power;
            }
            leftFront =  turn;
            rightFront = -turn;
            leftRear =  turn;
            rightRear =  - turn;

            frontLeftMotor.setPower(leftFront);
            backLeftMotor.setPower(leftRear);
            frontRightMotor.setPower(rightFront);
            backRightMotor.setPower(rightRear);

            sleep(100);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            if (degrees > 0)
                diff = (degrees+currentAngle); // needs to move this much: diff
            else
                diff = (-degrees-currentAngle); // needs to move this much: diff
            telemetry.addData("current degrees", currentAngle);
            telemetry.addData("diff", diff);
            telemetry.update();
        }

        //while (Math.abs(diff)>2) { // while there is more than 2 degree to move
        do {
            power2 = pidRotate.performPID(getAngle());

            sign = diff / Math.abs(diff);
            if (degrees > 0) {
                // increase the power to increase turning speed
                turn = sign * power2; //turn = sign*0.3; // by Bo
            } else {
                turn = -sign * power2;
            }

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
            if (degrees > 0)
                diff = (degrees+currentAngle); // needs to move this much: diff
            else
                diff = (-degrees-currentAngle); // needs to move this much: diff
            telemetry.addData("current degrees", currentAngle);
            telemetry.addData("diff", diff);
            telemetry.update();
        } while (opModeIsActive() && !pidRotate.onTarget());

        frontLeftMotor.setPower(0); //brake
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        rotated = getAngle();

        if (debugMode) {
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 turn rotation", rotated);
            telemetry.update();
        }
        
        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private void RotateClockwise(int angle, double power) {
        rotate(angle,power);
    }
    private void RotateCounterClockwise(int angle, double power) {
        rotate(-angle,power);
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