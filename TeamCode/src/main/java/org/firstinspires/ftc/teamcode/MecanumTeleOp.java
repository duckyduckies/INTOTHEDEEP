package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    private final static double ARM_POWER = 0.3;
    private final static double ARM_POWER_TO_TARGET = 0.5;
    private final static int ARM_INITIAL_POSITION = 0;
    private final static int ARM_STEP = 25;
    private final static int ARM_UPPER_LIMIT = 1100;
    private final static int ARM_LOWER_LIMIT = -1100;

    private ElapsedTime runtime = new ElapsedTime();
    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    boolean debug_mode = true;

    private class DriveThread extends Thread
    {
        public DriveThread()
        {
            this.setName("DriveThread");
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
            int eqVer = 0;
            frontLeftPower_mod = DcMotorPowerModifierAdv(frontLeftPower, eqVer);
            backLeftPower_mod = DcMotorPowerModifierAdv(backLeftPower, eqVer);
            frontRightPower_mod = DcMotorPowerModifierAdv(frontRightPower, eqVer);
            backRightPower_mod = DcMotorPowerModifierAdv(backRightPower, eqVer);

            frontLeftMotor.setPower(frontLeftPower_mod * powerScale);
            backLeftMotor.setPower(backLeftPower_mod * powerScale);
            frontRightMotor.setPower(frontRightPower_mod * powerScale);
            backRightMotor.setPower(backRightPower_mod * powerScale);

            /*if (debug_mode) {
                telemetry.addData("frontLeftPower: ", frontLeftPower);
                telemetry.addData("backLeftPower: ", backLeftPower);
                telemetry.addData("frontRightPower: ", frontRightPower);
                telemetry.addData("backRightPower: ", backRightPower);
                telemetry.addData("frontLeftPower_mod: ", frontLeftPower_mod);
                telemetry.addData("backLeftPower_mod: ", backLeftPower_mod);
                telemetry.addData("frontRightPower_mod: ", frontRightPower_mod);
                telemetry.addData("backRightPower_mod: ", backRightPower_mod);
            }
            */
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            try
            {
                double x,y,rx;
                while (opModeIsActive() && !isInterrupted())
                {
                    /***************** 1. Mecanum Drivetrain *****************/
                    y = gamepad1.left_stick_x; // Remember, Y stick value is reversed
                    x = -gamepad1.left_stick_y; // Counteract imperfect strafing
                    rx = gamepad1.right_stick_x;

                    move(y,x,rx,1);

                    if (gamepad1.dpad_up) {
                        move(0, 1,0,0.5);
                    }
                    else if (gamepad1.dpad_down) {
                        move(0, -1,0,0.5);
                    }
                    else if (gamepad1.dpad_right) {
                        move(1, 0,0,0.7);
                    }
                    else if (gamepad1.dpad_left) {
                        move(-1, 0,0,0.7);
                    }
                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {telemetry.addData("%s interrupted", this.getName());}
            // an error occurred in the run loop.
            catch (Exception e) {e.printStackTrace();}
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // CONFIGURATION--------------------------------------------------------------
        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

        waitForStart();
        /***************** 1. Mecanum Drivetrain *****************/
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        Thread  driveThread = new DriveThread();
        driveThread.start();


        /***************** 2. Viper Slides *****************/
        DcMotor slideMotor = hardwareMap.dcMotor.get("SlideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int slidePosition = 0;
        /***************** 3. Arm *****************/
        DcMotor armMotor = hardwareMap.dcMotor.get("ArmMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int armPosition = ARM_INITIAL_POSITION;
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER_TO_TARGET);
        /***************** 4. Wrist *****************/
        double wristPosition = 1; // default position is down
        Servo wristServo = hardwareMap.servo.get("WristServo");
        wristServo.setPosition(wristPosition);

        /***************** 5. Claw Intake *****************/
        CRServo intakeServoR = hardwareMap.crservo.get("IntakeServoR");
        CRServo intakeServoL = hardwareMap.crservo.get("IntakeServoL");
        intakeServoR.setPower(0);
        intakeServoL.setPower(0);

        // Intake state:
        // 0: Not rotating
        // 1: Intake
        // 2: Outake
        int intakePressed = 0;
        //int intakedirection = 0;

        /***************** 6. MiSUMi Slides *****************/
        Servo extendServoR = hardwareMap.servo.get("ExtendServoR");
        Servo extendServoL = hardwareMap.servo.get("ExtendServoL");
        extendServoR.setPosition(1);
        extendServoL.setPosition(0);
        //double slideExtendR = 1;
        //double slideExtendL = 0;
        double slideExtend = 1;

        /***************** 7. Lead Screw *****************/
        DcMotor LSMotorR = hardwareMap.dcMotor.get("LSMotorR");
        DcMotor LSMotorL = hardwareMap.dcMotor.get("LSMotorL");
        int LSPositionL = 0;
        int LSPositionR = 0;

        /***************** 8. Color Sensor *****************/
        final float[] hsvValues = new float[3];
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

        if (isStopRequested()) return;
//OP MODE CODE-------------------------------------------------------------------------
        while (opModeIsActive()) {
            // Click "back" button to toggle the debug view

            if (gamepad1.back || gamepad2.back) {
                debug_mode = !debug_mode;
            }

            /***************** 8. Color Sensor *****************/
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 2) {
                telemetry.addData("sample detected", 0);
                gamepad1.runRumbleEffect(rumbleEffect);
            }
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);


            /***************** 1. Mecanum Drivetrain *****************/

            //moved to move function

            /***************** 2. Viper Slides *****************/

            slidePosition = slideMotor.getCurrentPosition();
            if (gamepad2.left_stick_y < 0) {
                //slideMotor.setPower(-gamepad2.left_stick_y);
                slidePosition = slidePosition + 150; // arm goes up
                slideMotor.setTargetPosition(slidePosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);
            } else if (gamepad2.left_stick_y > 0 && slidePosition > 0) {
                //slideMotor.setPower(-gamepad2.left_stick_y);
                slidePosition = slidePosition - 150;
                slideMotor.setTargetPosition(slidePosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);
            }  else {
                //slideMotor.setPower(0);
            }
            if (gamepad2.left_stick_button){
                slideMotor.setPower(0);
            }
            /*
            if (gamepad1.y) {
                slideMotor.setTargetPosition(500);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.3);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3) && slideMotor.isBusy()) {
                }
            }
            if (debug_mode) {
                telemetry.addData("slidePosition: ", slidePosition);
            }

            */
            /***************** 3. Arm *****************/
            //armMotor.setPower(-gamepad2.right_stick_y/4);
            armPosition=armMotor.getCurrentPosition();
            if (gamepad2.right_stick_y < 0 && armPosition < ARM_UPPER_LIMIT) { // joystick above the origin
                armPosition = armPosition + ARM_STEP; // arm goes up
                /*
                if (slidePosition <= 2000) {
                    armPosition = 0;
                }
                 */
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
            } else if (gamepad2.right_stick_y > 0 && armPosition > ARM_LOWER_LIMIT) { // joystick below the origin
                armPosition = armPosition - ARM_STEP;
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
            } else {
                //armMotor.setPower(0); // RUN_WITHOUT_ENCODER mode
            }
            if (debug_mode) {
                telemetry.addData("armPosition:", armPosition);
            }
            if (gamepad1.x) {
                armMotor.setTargetPosition(-500);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
            }
            if (gamepad2.dpad_left) {
                armMotor.setTargetPosition(500);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
                extendServoR.setPosition(1);
                extendServoL.setPosition(0);
                wristServo.setPosition(0);
            }
            /***************** 4. Wrist *****************/
            if (gamepad2.dpad_down) { // wrist down
                if (wristPosition <= 1) {
                    wristPosition += 0.02;
                    wristServo.setPosition(wristPosition);
                }
            }
            if (gamepad2.dpad_up) {
                if (wristPosition >= 0) { // wrist up
                    wristPosition -= 0.02;
                    wristServo.setPosition(wristPosition);
                }
            }
            if (debug_mode) {
                telemetry.addData("wristPostiion:", wristPosition);
            }

            /***************** 5. Claw Intake *****************/
            if (intakePressed == 1) {
                if (gamepad2.right_bumper) {//brake
                    intakePressed = 0;
                    intakeServoR.setPower(0);
                    intakeServoL.setPower(0);
                }
                else if (gamepad2.left_bumper){//outtake
                    intakePressed = 2;
                    intakeServoR.setPower(-1.0);
                    intakeServoL.setPower(1.0);
                }
            } else if (intakePressed == 2) {
                if (gamepad2.left_bumper) {//brake
                    intakePressed = 0;
                    intakeServoR.setPower(0);
                    intakeServoL.setPower(0);
                }
                else if (gamepad2.right_bumper) { //intake
                    intakePressed = 1;
                    intakeServoR.setPower(1.0);
                    intakeServoL.setPower(-1.0);
                }
            } else if (gamepad2.right_bumper) { //intake
                intakePressed = 1;
                intakeServoR.setPower(1.0);
                intakeServoL.setPower(-1.0);
            } else if (gamepad2.left_bumper) { //outtake
                intakePressed = 2;
                intakeServoR.setPower(-1.0);
                intakeServoL.setPower(1.0);
            }

            /*
            if (gamepad2.right_bumper) {
                if (intakedirection == 1) {
                    intakedirection = 0;
                } else {
                    intakedirection = 1;
                }
            } else if (gamepad2.left_bumper) {
                if (intakedirection == -1) {
                    intakedirection = 0;
                } else {
                    intakedirection = -1;
                }
            }

            if (intakedirection == 1){
                intakeServoR.setPower(-1.0);
                intakeServoL.setPower(1.0);
            } else if (intakedirection == -1){
                intakeServoR.setPower(1.0);
                intakeServoL.setPower(-1.0);
            } else {
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                if (!((intakedirection == 0)||(intakedirection == -1)||(intakedirection == 1))) {
                    intakedirection = 0;
                }
            }
             */

            /***************** 6. MiSUMi Slides *****************/
            /*
            slideExtendR = extendServoR.getPosition();
            slideExtendL = extendServoL.getPosition();
            if (gamepad2.right_bumper) { // Misumi slide down
                if (slideExtendR <= 1) {
                    slideExtendR = slideExtendR + 0.05;
                    slideExtendL = slideExtendL - 0.05;
                    extendServoR.setPosition(slideExtendR);
                    extendServoL.setPosition(slideExtendL);
                }
            }
            if (gamepad2.left_bumper) {
                if (slideExtendR >= 0) { // Misumi slide up
                    slideExtendR = slideExtendR - 0.05;
                    slideExtendL = slideExtendL + 0.05;
                    extendServoR.setPosition(slideExtendL);
                    extendServoL.setPosition(slideExtendR);
                }
            }
             */

            //CY CHANGE--------------------------------Changed to variable speed + position, controls changed from bumper to trigger---------------------------------
            slideExtend = extendServoR.getPosition();
            if ((gamepad2.right_trigger > 0.1) && (slideExtend < 1)) {  // (armPosition < 0)
                slideExtend = slideExtend + 0.1 * (gamepad2.right_trigger/3);
                if (slideExtend>1) slideExtend=1;
                extendServoR.setPosition(slideExtend);
                extendServoL.setPosition(-slideExtend + 1);
            } else if (gamepad2.left_trigger > 0.1 && (slideExtend > 0)) {
                slideExtend = slideExtend - 0.1 * (gamepad2.left_trigger/3);
                if (slideExtend<0) slideExtend=0;
                extendServoR.setPosition(slideExtend);
                extendServoL.setPosition(-slideExtend + 1);
            }
            if (debug_mode) {
                telemetry.addData("slideExtendR:", slideExtend);
                telemetry.addData("slideExtendL:", 1-slideExtend);
            }


            /***************** 7. Lead Screw *****************/
            LSPositionR = LSMotorR.getCurrentPosition();
            LSPositionL = LSMotorL.getCurrentPosition();
            if (gamepad1.right_bumper && (LSPositionR < 5700 || LSPositionL < 5700)) {
                LSMotorR.setPower(1);
                LSMotorL.setPower(1);
            } else if (gamepad1.left_bumper && (LSPositionR > 0 || LSPositionL > 0)) {
                LSMotorR.setPower(-1);
                LSMotorL.setPower(-1);
            } else {
                LSMotorR.setPower(0);
                LSMotorL.setPower(0);
            }
            if (gamepad1.right_trigger > 0.25) {
                LSMotorR.setTargetPosition(5700);
                LSMotorL.setTargetPosition(5700);
            }
            else if (gamepad1.left_trigger > 0.25) {
                LSMotorR.setTargetPosition(0);
                LSMotorL.setTargetPosition(0);
            }
            if (debug_mode) {
                telemetry.addData("LSPositionR:", LSPositionR);
                telemetry.addData("LSPositionL:", LSPositionL);
            }
            /***************** Preset Buttons *****************/
            /*
            if (gamepad2.a){ // intake position
                wristPosition=(0.55);
                wristServo.setPosition(wristPosition);
                armMotor.setTargetPosition(-720);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
            }

            if (gamepad2.b) { // outtake at high basket
                wristPosition=(0);
                wristServo.setPosition(wristPosition);
                armMotor.setTargetPosition(1103);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
                extendServoR.setPosition(0.45);
                extendServoL.setPosition(0.55);
                slideMotor.setTargetPosition(2000);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.3);
            }
            if (gamepad2.x) {
                wristPosition=(0);
                wristServo.setPosition(wristPosition);
                armMotor.setTargetPosition(1103);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                extendServoR.setPosition(0.25);
                extendServoL.setPosition(0.75);
                slideMotor.setTargetPosition(2000);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.3);
            }
            if (gamepad2.y) {
                slideMotor.setTargetPosition(2000);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.3);
                armMotor.setTargetPosition(-720);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                wristPosition = 0;
                wristServo.setPosition(wristPosition);

            }
            */
            telemetry.update();

        } // end of while loop
    } // end of opMode

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
}