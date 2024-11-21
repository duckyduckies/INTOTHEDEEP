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

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        // CONFIGURATION--------------------------------------------------------------
        boolean debug_mode = true;
        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //DcMotor slideMotorR = hardwareMap.dcMotor.get("SlideMotorR");
        DcMotor slideMotor = hardwareMap.dcMotor.get("SlideMotor");
        //slideMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor armMotor = hardwareMap.dcMotor.get("ArmMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double wristPosition = 1; // default position is down
        Servo wristServo = hardwareMap.servo.get("WristServo");
        wristServo.setPosition(wristPosition);
/*
        double armPositionR = 0.6; //midpoint
        double armPositionL = 0.4; //midpoint
        Servo armServoR = hardwareMap.servo.get("ArmServoR");
        Servo armServoL = hardwareMap.servo.get("ArmServoL");
        armServoR.setPosition(armPositionR);
        armServoL.setPosition(armPositionL);
*/
        /*

        CRServo armServoR = hardwareMap.crservo.get("ArmServoR");
        CRServo armServoL = hardwareMap.crservo.get("ArmServoL");
        armServoR.setPower(0);
        armServoL.setPower(0);
*/
        CRServo intakeServoR = hardwareMap.crservo.get("IntakeServoR");
        CRServo intakeServoL = hardwareMap.crservo.get("IntakeServoL");
        intakeServoR.setPower(0);
        intakeServoL.setPower(0);

        Servo extendServoR = hardwareMap.servo.get("ExtendServoR");  
        Servo extendServoL = hardwareMap.servo.get("ExtendServoL");
        extendServoR.setPosition(1);
        extendServoL.setPosition(0);
        double slideExtend = 0;
        int intakedirection = 0;
        final float[] hsvValues = new float[3];
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

        DcMotor LSMotorR = hardwareMap.dcMotor.get("LSMotorR");
        DcMotor LSMotorL = hardwareMap.dcMotor.get("LSMotorL");

        waitForStart();

        if (isStopRequested()) return;
//OP MODE CODE-------------------------------------------------------------------------
        while (opModeIsActive()) {
            // Click "back" button to toggle the debug view

            if (gamepad1.back || gamepad2.back) {
                debug_mode = !debug_mode;
            }

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            // Mecanum Drivetrain
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

//          double speedReduction = 0.5;

            //frontLeftPower=Math.pow(Math.tanh(frontLeftPower)/Math.tanh(1),3);
//            frontLeftMotor.setPower(frontLeftPower*speedReduction);
            //backLeftPower=Math.pow(Math.tanh(backLeftPower)/Math.tanh(1),3);
//            backLeftMotor.setPower(backLeftPower*speedReduction);
            //frontRightPower=Math.pow(Math.tanh(frontRightPower)/Math.tanh(1),3);
//            frontRightMotor.setPower(frontRightPower*speedReduction);
            //backRightPower=Math.pow(Math.tanh(backRightPower)/Math.tanh(1),3);
//            backRightMotor.setPower(backRightPower*speedReduction);
/*
            double frontLeftPower_mod = Math.pow(Math.tanh(frontLeftPower)/Math.tanh(1),3);
            double backLeftPower_mod = Math.pow(Math.tanh(backLeftPower)/Math.tanh(1),3);
            double frontRightPower_mod = Math.pow(Math.tanh(frontRightPower)/Math.tanh(1),3);
            double backRightPower_mod = Math.pow(Math.tanh(backRightPower)/Math.tanh(1),3);

            // modify power by using DcMotorPowerModifier()
            double frontLeftPower_mod = DcMotorPowerModifier(frontLeftPower);
            double backLeftPower_mod = DcMotorPowerModifier(backLeftPower);
            double frontRightPower_mod = DcMotorPowerModifier(frontRightPower);
            double backRightPower_mod = DcMotorPowerModifier(backRightPower);
*/
            // modify power by using DcMotorPowerModifierAdv()
            int eqVer = 1;
            double frontLeftPower_mod = DcMotorPowerModifierAdv(frontLeftPower, eqVer);
            double backLeftPower_mod = DcMotorPowerModifierAdv(backLeftPower, eqVer);
            double frontRightPower_mod = DcMotorPowerModifierAdv(frontRightPower, eqVer);
            double backRightPower_mod = DcMotorPowerModifierAdv(backRightPower, eqVer);

            frontLeftMotor.setPower(frontLeftPower_mod);
            backLeftMotor.setPower(backLeftPower_mod);
            frontRightMotor.setPower(frontRightPower_mod);
            backRightMotor.setPower(backRightPower_mod);

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

            // Wrist
            /*
            if (gamepad2.dpad_up) {
                wristServo.setPosition(1);
            }
            if (gamepad2.dpad_down) {
                wristServo.setPosition(0);
            }
            */

            //***** change from while to if - not tested
            if (gamepad2.dpad_down) { // wrist down
                if (wristPosition <= 1) {
                    wristPosition += 0.05;
                    wristServo.setPosition(wristPosition);
                }
            }
            //***** change from while to if - not tested
            if (gamepad2.dpad_up) {
                if (wristPosition >= 0) { // wrist up
                    wristPosition -= 0.05;
                    wristServo.setPosition(wristPosition);
                }
            }
            if (debug_mode) {
                telemetry.addData("wristPostiion:", wristPosition);
            }

            /*
            if (gamepad2.left_stick_y > 0.3) {
                if (armPositionR <= 1 && armPositionL >= 0) {
                    armPositionR += 0.1;
                    armPositionL -= 0.1;
                    armServoR.setPosition(armPositionR);
                    armServoL.setPosition(armPositionL);
                }
            }
            if (gamepad2.left_stick_y < -0.3) {
                if (armPositionR >= 0 && armPositionL <= 1) {
                    armPositionR -= 0.1;
                    armPositionL += 0.1;
                    armServoR.setPosition(armPositionR);
                    armServoL.setPosition(armPositionL);
                }
            }


            if (gamepad1.x) {
                armPositionR = 0.5;
                armPositionL = 0.5;
                armServoR.setPosition(armPositionR);
                armServoL.setPosition(armPositionL);
            }

            if (debug_mode) {
                telemetry.addData("armPositionR:", armPositionR);
                telemetry.addData("armPositionL:", armPositionL);

            }
             */

            //***** change from servos to CRServos and to motor
            armMotor.setPower(-gamepad2.right_stick_y/4);
            if (debug_mode) {
                telemetry.addData("armPosition:", armMotor.getCurrentPosition());
            }
// CY CHANGE----------------------TRIGGER FUNCTIONS NOW SET TO BUMPER--------------------------------//CY CHANGE--------------------------------------ONE CLICK CONTINUOUS BUMPER CONTROLS----------------------------
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


            if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 2) {
                telemetry.addData("sample detected", 0);
                gamepad1.runRumbleEffect(rumbleEffect);
                gamepad2.runRumbleEffect(rumbleEffect);
            }
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            if (gamepad1.y) {
                slideMotor.setTargetPosition(500);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.2);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3) && slideMotor.isBusy()) {
                }
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


            if (gamepad1.right_bumper) {
                LSMotorR.setPower(0.5);
                LSMotorL.setPower(0.5);
            } else if (gamepad1.left_bumper) {
                LSMotorR.setPower(-0.5);
                LSMotorL.setPower(-0.5);
            } else {
                LSMotorR.setPower(0);
                LSMotorL.setPower(0);
            }

            if (gamepad2.left_stick_y < 0) {
                slideMotor.setPower(-0.5);
            } else if (gamepad2.left_stick_y > 0) {
                slideMotor.setPower(-gamepad2.left_stick_y);
            } else {
                slideMotor.setPower(0);
            }

//CY CHANGE--------------------------------Changed to variable speed + position, controls changed from bumper to trigger---------------------------------
            if ((gamepad2.right_trigger > 0.1) && (slideExtend > 0)) {
                slideExtend = slideExtend + 0.1 * gamepad2.right_trigger;
                extendServoR.setPosition(slideExtend);
                extendServoL.setPosition(-slideExtend + 1);
            } else if (gamepad2.left_trigger > 0.1 && (slideExtend < 1)) {
                slideExtend = slideExtend - 0.1 * gamepad2.left_trigger;
                extendServoR.setPosition(slideExtend);
                extendServoL.setPosition(-slideExtend + 1);
            } else {
                extendServoL.setPosition(slideExtend);
                extendServoR.setPosition(-slideExtend + 1);
            }
            if (gamepad2.a){ // intake
                armMotor.setTargetPosition(-500);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);

                wristPosition=(0.4);
                wristServo.setPosition(wristPosition);

                armMotor.setTargetPosition(-1100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                extendServoR.setPosition(0);
                extendServoL.setPosition(1);
            }
            if (gamepad2.b) { // outtake at high basket
                extendServoR.setPosition(0);
                extendServoL.setPosition(1);
                // still missing arm
                /*
                slideMotor.setTargetPosition(0);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.3);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3) && slideMotor.isBusy()) {
                }
                */
                wristServo.setPosition(0);
            }
            telemetry.update();

        }
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
}