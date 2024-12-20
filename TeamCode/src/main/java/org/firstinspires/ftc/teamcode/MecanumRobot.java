package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MecanumRobot {
    private ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private boolean debugMode = true;

    /***************** 0. IMU *****************/
    IMU imu;
    public static PIDController           pidRotate = new PIDController(.003, .00003, 0);
    //public static pidDrive;
    private Orientation lastAngles = new Orientation();
    private double                  globalAngle;
    private double                 rotated;

    /***************** 1. Mecanum Drivetrain *****************/
    public static boolean robotCentricDrive = true;
    private final static int DRIVETRAIN_POWER_MODIFIER_EQ_VER = 0;
    public static double DPAD_FORWARD_BACKWARD_POWER_RATIO = 0.4;
    public static double DPAD_SIDEWAY_POWER_RATIO = 0.8;
    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    /***************** 2. Viper Slides *****************/ //28 inch,
    //private final static double VIPER_SLIDES_POWER = 0.75;
    private final static double VIPER_SLIDES_POWER_MANUAL = 1;
    public static double VIPER_SLIDES_POWER_PRESET = 1;
    public static double VIPER_SLIDES_POWER_PRESET_DOWN = 0.8;
    private final static int VIPER_SLIDES_INITIAL_POSITION = 0;
    private final static int VIPER_SLIDES_STEP = 200;
    private final static int VIPER_SLIDES_UPPER_LIMIT = 5000;
    private final static int VIPER_SLIDES_LOWER_LIMIT = 0;
    private final static int VIPER_SLIDES_OFF_THRESHOLD = 50;
    DcMotor slideMotorR;
    DcMotor slideMotorL;
    int slidePositionR = VIPER_SLIDES_INITIAL_POSITION;
    int slidePositionL = VIPER_SLIDES_INITIAL_POSITION;
    /***************** 3. Arm *****************/

    //private final static double ARM_POWER = 0.5;
    private final static double ARM_POWER_MANUAL = 0.6;
    public static double ARM_POWER_PRESET = 0.9;
    private final static int ARM_INITIAL_POSITION = 0;
    private final static int ARM_STEP = 50;
    private final static int ARM_UPPER_LIMIT = 350;
    private final static int ARM_LOWER_LIMIT = -1400;
    private final static int ARM_MISUMI_RETRACT_THRESHOLD_L = -1100;
    private final static int ARM_WRIST_RETRACT_THRESHOLD_L = -900;
    DcMotor armMotor;
    int armPosition = ARM_INITIAL_POSITION;
    /***************** 4. Wrist *****************/
    private final static double WRIST_STEP = 0.02;
    private final static double WRIST_UP = 0.38;
    private final static double WRIST_DOWN = 1;
    double wristPosition = WRIST_DOWN; // default position is down
    Servo wristServo;

    /***************** 5. Claw Intake *****************/
    private final static int CLAW_INITIAL_STATE = 0;
    private final static double CLOCKWISE_POWER = -1.0;
    private final static double COUNTER_CLOCKWISE_POWER = 1.0;
    // Intake state:
    // 0: NotOutake
    int clawState = CLAW_INITIAL_STATE;
    //int intakedirection = 0;
    CRServo clawServoR;
    CRServo clawServoL;

    /***************** 6. MiSUMi Slides *****************/
    private final static double MISUMI_STEP_RATIO_1 = 0.1; // CY defines it
    private final static double MISUMI_EXTEND_LIMIT_R = 0.35;
    private final static double MISUMI_RETRACT_LIMIT_R = 0;
    private final static double MISUMI_STEP_RATIO_2 = MISUMI_EXTEND_LIMIT_R/1; // CC defines it
    Servo extendServoR;
    Servo extendServoL;
    double slideExtendR = MISUMI_RETRACT_LIMIT_R;

    /***************** 7. Lead Screw *****************/
    private final static double LEAD_SCREW_POWER = 0.7;
    private final static double LEAD_SCREW_POWER_PRESET = 1;
    private final static int LEAD_SCREW_INITIAL_POSITION = 0;
    //private final static int LEAD_SCREW_STEP = 150;
    private final static int LEAD_SCREW_UPPER_LIMIT = 34000;
    private final static int LEAD_SCREW_LOWER_LIMIT = 0;
    private final static int LEAD_SCREW_OFF_THRESHOLD = 300;

    DcMotor LSMotorR;
    //DcMotor LSMotorL = myOpMode.hardwareMap.dcMotor.get("LSMotorL");
    int LSPositionL = LEAD_SCREW_INITIAL_POSITION;
    int LSPositionR = LEAD_SCREW_INITIAL_POSITION;
    int LSState = 0;
    /***************** 8. Color Sensor *****************/
    public static boolean TEAM_COLOR_RED = false; // 224, 18, 76, 154
    private final static int BLUE_HUE = 224;
    private final static int RED_HUE = 18;
    private final static int YELLOW_HUE = 76;
    private final static int NO_SAMPLE_HUE = 154;
    ColorSensor colorSensor;
    /***************** 9. LED *****************/
    RevBlinkinLedDriver blinkinLedDriver;
    private final static RevBlinkinLedDriver.BlinkinPattern DEFAULT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GRAY;
    private final static RevBlinkinLedDriver.BlinkinPattern YELLOW_PATTERN = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private final static RevBlinkinLedDriver.BlinkinPattern RED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;
    private final static RevBlinkinLedDriver.BlinkinPattern BLUE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    protected enum DisplayKind {
        MANUAL,
        AUTO
    }
    private MecanumTeleOp.DisplayKind displayKind;
    private String currentPattern;

    public String getBlinkinLEDDisplayPattern() {
        return currentPattern;
    }
    protected void setBlinkinLEDDisplayPattern(RevBlinkinLedDriver.BlinkinPattern pattern)
    {
        blinkinLedDriver.setPattern(pattern);
        currentPattern = pattern.toString();
    }
    /***************** 10. FTC Dashboard *****************/
    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) {
                continue;
            }

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }
        }
    }
    /***************** Preset Buttons *****************/
    public static double INTAKE_PRESET_WRIST_POS = 0.52;
    public static int INTAKE_PRESET_ARM_POS = -1400;
    public static int OUTTAKE_PRESET_HIGH_BASKET_VIPER_POS = 3600;
    public static int OUTTAKE_PRESET_LOW_BASKET_VIPER_POS = 950;
    public static double OUTTAKE_PRESET_WRIST_POS = 0.38;
    public static int OUTTAKE_PRESET_ARM_POS = 350;
    public static double OUTTAKE_PRESET_CHAMBER_WRIST_POS = 0.4;
    public static int OUTTAKE_PRESET_HIGH_CHAMBER_VIPER_1 = 800;
    public static int OUTTAKE_PRESET_CHAMBER_VIPER_VIPER_1 = 766;
    public static int LS_ABOVE_LOWER_RUNG = 34000;
    public static int LS_LOWER_RUNG = 8000;
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public MecanumRobot (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize()
    {
        /***************** 0. IMU *****************/
        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        if (robotCentricDrive) {
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
        } else {
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
        }
        imu.resetYaw();

        /***************** 1. Mecanum Drivetrain *****************/
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor = myOpMode.hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = myOpMode.hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = myOpMode.hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = myOpMode.hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /***************** 2. Viper Slides *****************/
        slideMotorR = myOpMode.hardwareMap.dcMotor.get("SlideMotorR");
        slideMotorL = myOpMode.hardwareMap.dcMotor.get("SlideMotorL");
        slideMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /***************** 3. Arm *****************/
        armMotor = myOpMode.hardwareMap.dcMotor.get("ArmMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /***************** 4. Wrist *****************/
        wristServo = myOpMode.hardwareMap.servo.get("WristServo");
        //wristServo.setPosition(wristPosition);

        ///*************** 5. Claw Intake
        clawServoR = myOpMode.hardwareMap.crservo.get("ClawServoR");
        clawServoL = myOpMode.hardwareMap.crservo.get("ClawServoL");
        clawServoR.setPower(0);
        clawServoL.setPower(0);

        /***************** 6. MiSUMi Slides *****************/
        extendServoR = myOpMode.hardwareMap.servo.get("ExtendServoR");
        extendServoL = myOpMode.hardwareMap.servo.get("ExtendServoL");
        //extendServoR.setPosition(slideExtendR);
        //extendServoL.setPosition(-slideExtendR+1);

        /***************** 7. Lead Screw *****************/
        LSMotorR = myOpMode.hardwareMap.dcMotor.get("LSMotorR");

        /***************** 8. Color Sensor *****************/
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to our ColorSensor object.
        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "ColorSensor");

        /***************** 9. LED *****************/
        displayKind = MecanumTeleOp.DisplayKind.MANUAL;
        blinkinLedDriver = myOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        setBlinkinLEDDisplayPattern(DEFAULT_PATTERN);

    }

    /**
     * Mecanum Drivetrain
     * Calculates the left/right front/rear motor powers required to achieve the requested
     * robot motions: ...
     * @param x
     * @param y
     * @param rx
     * @param powerScale
     */
    public void move(double x, double y, double rx, double powerScale) {
        double denominator,frontLeftPower,backLeftPower,frontRightPower,backRightPower;
        double frontLeftPower_mod,backLeftPower_mod,frontRightPower_mod,backRightPower_mod;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the rang e [-1, 1]
        x = x * 1.1;
        denominator = Math.max(Math.abs(y) + Math.abs(x)+ Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        // modify power by using DcMotorPowerModifierAdv()
        int eqVer = DRIVETRAIN_POWER_MODIFIER_EQ_VER;
        frontLeftPower_mod = DcMotorPowerModifierAdv(frontLeftPower, eqVer);
        backLeftPower_mod = DcMotorPowerModifierAdv(backLeftPower, eqVer);
        frontRightPower_mod = DcMotorPowerModifierAdv(frontRightPower, eqVer);
        backRightPower_mod = DcMotorPowerModifierAdv(backRightPower, eqVer);

        frontLeftMotor.setPower(frontLeftPower_mod * powerScale);
        backLeftMotor.setPower(backLeftPower_mod * powerScale);
        frontRightMotor.setPower(frontRightPower_mod * powerScale);
        backRightMotor.setPower(backRightPower_mod * powerScale);

            /*if (debugMode) {
                myOpMode.telemetry.addData("frontLeftPower_mod: ", frontLeftPower_mod * powerScale);
                myOpMode.telemetry.addData("backLeftPower_mod: ", backLeftPower_mod * powerScale);
                myOpMode.telemetry.addData("frontRightPower_mod: ", frontRightPower_mod * powerScale);
                myOpMode.telemetry.addData("backRightPower_mod: ", backRightPower_mod * powerScale);
                myOpMode.telemetry.update();
            }
            */
    }

    public void fieldCentricDriveMove(double x, double y, double rx, double botHeading, double powerScale) {
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * powerScale);
        backLeftMotor.setPower(backLeftPower * powerScale);
        frontRightMotor.setPower(frontRightPower * powerScale);
        backRightMotor.setPower(backRightPower * powerScale);

            /*if (debugMode) {
                myOpMode.telemetry.addData("frontLeftPower: ", frontLeftPower * powerScale);
                myOpMode.telemetry.addData("backLeftPower: ", backLeftPower * powerScale);
                myOpMode.telemetry.addData("frontRightPower: ", frontRightPower * powerScale);
                myOpMode.telemetry.addData("backRightPower: ", backRightPower * powerScale);
            }
            */
    }

    // angle: + clockwise; - counter-clockwise
    // power: always + here
    // This function only supports moving MORE THAN 2 degree
    public void rotate(int degrees, double power) {

        //
        // This part comes from Portland State University
        //
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

        //
        // This part comes from last season.
        // There were two rotate functions back then: clockwise & counter-clockwise
        // To be consistent with Portland State Univ., two functions are combined.
        // Positive input degree means clockwise;
        // Negative input degree means counter-clockwise.
        //
        // For a PID function to determine the next power used to rotate,
        // the degree must not be 0, so first rotate at least 2 degree to get off zero.
        //
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
        while (myOpMode.opModeIsActive() && Math.abs(diff)>=absDiff-2) {
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

            myOpMode.sleep(100);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            if (degrees > 0)
                diff = (degrees+currentAngle); // needs to move this much: diff
            else
                diff = (-degrees-currentAngle); // needs to move this much: diff
            myOpMode.telemetry.addData("target degrees", degrees);
            myOpMode.telemetry.addData("current degrees", currentAngle);
            myOpMode.telemetry.addData("diff", diff);
            myOpMode.telemetry.update();
        }

        //while (Math.abs(diff)>2) { // while there is more than 2 degree to move
        do {
            //power2 = pidRotate.performPID(getAngle());
            power2 = power;
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
            myOpMode.telemetry.addData("target degrees", degrees);
            myOpMode.telemetry.addData("current degrees", currentAngle);
            myOpMode.telemetry.addData("diff", diff);
            myOpMode.telemetry.update();
        } while (myOpMode.opModeIsActive() && Math.abs(diff)>=absDiff-2);
        //while (myOpMode.opModeIsActive() && !pidRotate.onTarget());

        frontLeftMotor.setPower(0); //brake
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        rotated = getAngle();

        if (debugMode) {
            myOpMode.telemetry.addData("1 imu heading", lastAngles.firstAngle);
            myOpMode.telemetry.addData("2 global heading", globalAngle);
            myOpMode.telemetry.addData("3 turn rotation", rotated);
            myOpMode.telemetry.update();
        }

        // wait for rotation to stop.
        myOpMode.sleep(500);

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
     * (Portland State University)
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * (Portland State University)
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
    public void highBasket() {
        armMotor.setTargetPosition(ARM_INITIAL_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER_PRESET);
        slideMotorR.setTargetPosition(OUTTAKE_PRESET_HIGH_BASKET_VIPER_POS); //10300
        slideMotorL.setTargetPosition(OUTTAKE_PRESET_HIGH_BASKET_VIPER_POS); //10300
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(VIPER_SLIDES_POWER_PRESET);
        slideMotorL.setPower(VIPER_SLIDES_POWER_PRESET);
        slideMotorR.setTargetPosition(OUTTAKE_PRESET_LOW_BASKET_VIPER_POS); //2850
        slideMotorL.setTargetPosition(OUTTAKE_PRESET_LOW_BASKET_VIPER_POS); //2850
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(VIPER_SLIDES_POWER_PRESET);
        slideMotorL.setPower(VIPER_SLIDES_POWER_PRESET);
        runtime.reset();
        while (myOpMode.opModeIsActive() && slideMotorR.isBusy() && slideMotorL.isBusy() && (runtime.milliseconds() < 500))
            myOpMode.idle();
        extendServoR.setPosition(MISUMI_EXTEND_LIMIT_R);
        extendServoL.setPosition(1 - MISUMI_EXTEND_LIMIT_R);
        wristServo.setPosition(OUTTAKE_PRESET_WRIST_POS);
        armMotor.setTargetPosition(OUTTAKE_PRESET_ARM_POS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER_PRESET);
    }
}