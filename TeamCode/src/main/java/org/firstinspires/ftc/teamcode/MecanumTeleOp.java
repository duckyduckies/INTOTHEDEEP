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
    private final static double TRIGGER_THRESHOLD = 0.25;

    /***************** 1. Mecanum Drivetrain *****************/

    private final static int DRIVETRAIN_POWER_MODIFIER_EQ_VER = 0;
    private final static double DPAD_FORWARD_BACKWARD_POWER_RATIO = 0.4;
    private final static double DPAD_SIDEWAY_POWER_RATIO = 0.8;

    /***************** 2. Viper Slides *****************/ //28 inch,
    private final static double VIPER_SLIDES_POWER = 0.75;
    private final static double VIPER_SLIDES_POWER_TO_TARGET = 1;
    private final static double VIPER_SLIDES_POWER_PRESET = 1;
    private final static double VIPER_SLIDES_POWER_PRESET_DOWN = 0.6;
    private final static int VIPER_SLIDES_INITIAL_POSITION = 0;
    private final static int VIPER_SLIDES_STEP = 200;
    private final static int VIPER_SLIDES_UPPER_LIMIT = 15000;
    private final static int VIPER_SLIDES_LOWER_LIMIT = 0;
    private final static int VIPER_SLIDES_OFF_THRESHOLD = 150;
    
    /***************** 3. Arm *****************/

    private final static double ARM_POWER = 0.5;
    private final static double ARM_POWER_TO_TARGET = 0.5;
    private final static double ARM_POWER_PRESET = 0.5;
    private final static int ARM_INITIAL_POSITION = 0;
    private final static int ARM_STEP = 100;
    private final static int ARM_UPPER_LIMIT = 500;
    private final static int ARM_LOWER_LIMIT = -1300;
    private final static int ARM_UPRIGHT_POSITION = 250;
    private final static int ARM_MISUMI_RETRACT_THRESHOLD_U = -750;
    private final static int ARM_MISUMI_RETRACT_THRESHOLD_L = -950;
    private final static int ARM_WRIST_RETRACT_THRESHOLD_U = -300;
    private final static int ARM_WRIST_RETRACT_THRESHOLD_L = 750;


    /***************** 4. Wrist *****************/
    private final static double WRIST_INITIAL_POSITION = 1;
    private final static double WRIST_STEP = 0.02;
    private final static double WRIST_UP = 0;
    private final static double WRIST_DOWN = 1;

    /***************** 5. Claw Intake *****************/
    private final static int CLAW_INITIAL_STATE = 0;
    private final static double CLOCKWISE_POWER = -1.0;
    private final static double COUNTER_CLOCKWISE_POWER = 1.0;

    /***************** 6. MiSUMi Slides *****************/
    private final static double MISUMI_INITIAL_POSITION_R = 0;
    private final static double MISUMI_INITIAL_POSITION_L = 1;
    private final static double MISUMI_STEP_RATIO_1 = 0.1; // CY defines it
    private final static double MISUMI_EXTEND_LIMIT_R = 0.3;
    private final static double MISUMI_RETRACT_LIMIT_R = 0;
    private final static double MISUMI_STEP_RATIO_2 = MISUMI_EXTEND_LIMIT_R/1; // CC defines it

    /***************** 7. Lead Screw *****************/
    private final static double LEAD_SCREW_POWER = 0.7;
    private final static double LEAD_SCREW_POWER_PRESET = 1;
    private final static int LEAD_SCREW_INITIAL_POSITION = 0;
    //private final static int LEAD_SCREW_STEP = 150;
    private final static int LEAD_SCREW_UPPER_LIMIT = 34000;
    private final static int LEAD_SCREW_LOWER_LIMIT = 0;
    private final static int LEAD_SCREW_OFF_THRESHOLD = 300;
    private final static int LS_ABOVE_LOWER_RUNG = 30000;
    private final static int LS_LOWER_RUNG = 8000;
    /***************** 8. Color Sensor *****************/
    
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
            int eqVer = DRIVETRAIN_POWER_MODIFIER_EQ_VER;
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
                        move(0, 1,0,DPAD_FORWARD_BACKWARD_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_down) {
                        move(0, -1,0,DPAD_FORWARD_BACKWARD_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_right) {
                        move(1, 0,0,DPAD_SIDEWAY_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_left) {
                        move(-1, 0,0,DPAD_SIDEWAY_POWER_RATIO);
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
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Thread  driveThread = new DriveThread();
        driveThread.start();


        /***************** 2. Viper Slides *****************/
        DcMotor slideMotor = hardwareMap.dcMotor.get("SlideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int slidePosition = VIPER_SLIDES_INITIAL_POSITION;
        /***************** 3. Arm *****************/
        DcMotor armMotor = hardwareMap.dcMotor.get("ArmMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int armPosition = ARM_INITIAL_POSITION;
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER_TO_TARGET);
        /***************** 4. Wrist *****************/
        double wristPosition = WRIST_INITIAL_POSITION; // default position is down
        Servo wristServo = hardwareMap.servo.get("WristServo");
        wristServo.setPosition(wristPosition);

        ///*************** 5. Claw Intake
        CRServo intakeServoR = hardwareMap.crservo.get("IntakeServoR");
        CRServo intakeServoL = hardwareMap.crservo.get("IntakeServoL");
        intakeServoR.setPower(0);
        intakeServoL.setPower(0);

        // Intake state:
        // 0: Not rotating
        // 1: Intake
        // 2: Outake
        int intakePressed = CLAW_INITIAL_STATE;
        //int intakedirection = 0;

        /***************** 6. MiSUMi Slides *****************/
        Servo extendServoR = hardwareMap.servo.get("ExtendServoR");
        Servo extendServoL = hardwareMap.servo.get("ExtendServoL");
        extendServoR.setPosition(0);
        extendServoL.setPosition(1);
        //double slideExtendR = 1;
        //double slideExtendL = 0;
        double slideExtendR = 0;

        /***************** 7. Lead Screw *****************/
        DcMotor LSMotorR = hardwareMap.dcMotor.get("LSMotorR");
        DcMotor LSMotorL = hardwareMap.dcMotor.get("LSMotorL");
        int LSPositionL = LEAD_SCREW_INITIAL_POSITION;
        int LSPositionR = LEAD_SCREW_INITIAL_POSITION;
        int LSState = 0;

        /***************** 8. Color Sensor *****************/
        final float[] hsvValues = new float[3];
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

        if (isStopRequested()) return;
//OP MODE CODE-------------------------------------------------------------------------
        while (opModeIsActive()) {
            // Click "back" button to toggle the debug view

            if (gamepad1.start || gamepad2.start) {
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

            // first priority
            if (gamepad2.left_stick_y < 0 && slidePosition <= VIPER_SLIDES_UPPER_LIMIT) { // slides go up
                //slideMotor.setPower(-gamepad2.left_stick_y);
                slidePosition = slidePosition + VIPER_SLIDES_STEP; // slides goes up
                slideMotor.setTargetPosition(slidePosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(VIPER_SLIDES_POWER_TO_TARGET);
            } else if (gamepad2.left_stick_y > 0 && slidePosition > VIPER_SLIDES_LOWER_LIMIT) {
                if (slidePosition < VIPER_SLIDES_OFF_THRESHOLD) { //turns off viper slides to prevent burning
                    slideMotor.setPower(0);
                    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    //slideMotor.setPower(-gamepad2.left_stick_y);
                    slidePosition = slidePosition - VIPER_SLIDES_STEP;
                    slideMotor.setTargetPosition(slidePosition);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotor.setPower(VIPER_SLIDES_POWER_TO_TARGET);
                }
            } else if (gamepad2.left_stick_button){//turns off viper slides
                slideMotor.setPower(0);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


            */
            if (debug_mode) {
                telemetry.addData("slidePosition: ", slidePosition);
            }
            /***************** 3. Arm *****************/
            //armMotor.setPower(-gamepad2.right_stick_y/4);
            armPosition=armMotor.getCurrentPosition();
            if (gamepad2.right_stick_y < 0 && armPosition <= ARM_UPPER_LIMIT) { // joystick above the origin; arm raises up
                // retracts misumi slides when the arm rotates up and leaves the floor (at -1100 )
                if (armPosition<=ARM_MISUMI_RETRACT_THRESHOLD_U && armPosition>=ARM_MISUMI_RETRACT_THRESHOLD_L){
                    extendServoR.setPosition(MISUMI_INITIAL_POSITION_R);
                    extendServoL.setPosition(MISUMI_INITIAL_POSITION_L);
                    intakeServoR.setPower(0);
                    intakeServoL.setPower(0);
                }
                if (armPosition<=ARM_WRIST_RETRACT_THRESHOLD_U && armPosition>=ARM_WRIST_RETRACT_THRESHOLD_L){
                    wristServo.setPosition(WRIST_DOWN);

                    intakeServoR.setPower(0);
                    intakeServoL.setPower(0);
                    intakePressed=0;
                }
                armPosition = armPosition + ARM_STEP; // arm goes up by one step
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);

            } else if (gamepad2.right_stick_y > 0 && armPosition >= ARM_LOWER_LIMIT) { // joystick below the origin; arm puts down
                if (armPosition<=ARM_WRIST_RETRACT_THRESHOLD_U && armPosition>=ARM_WRIST_RETRACT_THRESHOLD_L){
                    wristServo.setPosition(0.52);
                }
                armPosition = armPosition - ARM_STEP; // arm goes down by one step
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
            } //else {
                //armMotor.setPower(0); // RUN_WITHOUT_ENCODER mode
            //}
            if (debug_mode) {
                telemetry.addData("armPosition:", armPosition);
            }
            if (armPosition > 500) {
                armMotor.setTargetPosition(499);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            /*
            // Test Arm
            if (gamepad1.x) {
                armMotor.setTargetPosition(-500);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
            }
            */

            /***************** 4. Wrist *****************/
            if (gamepad2.dpad_down) { 
                if (wristPosition <= WRIST_DOWN) { // wrist down
                    wristPosition += WRIST_STEP;
                    wristServo.setPosition(wristPosition);
                }
            }
            else if (gamepad2.dpad_up) {
                if (wristPosition >= WRIST_UP) { // wrist up
                    wristPosition -= WRIST_STEP;
                    wristServo.setPosition(wristPosition);
                }
            }
            if (debug_mode) {
                telemetry.addData("wristPostiion:", wristPosition);
            }

            /***************** 5. Claw Intake *****************/
            // Intake state:
            // 0: Not rotating
            // 1: Intake
            // 2: Outake
            if (intakePressed == 1) { //intaking
                if (gamepad2.dpad_right) { //brake
                    intakePressed = 0;
                    intakeServoR.setPower(0);
                    intakeServoL.setPower(0);
                    idle();
                }
                else if (gamepad2.dpad_left){ //outtake
                    intakePressed = 2;
                    intakeServoR.setPower(CLOCKWISE_POWER);
                    intakeServoL.setPower(COUNTER_CLOCKWISE_POWER);
                    idle();
                }
            } else if (intakePressed == 2) { //outtaking
                if (gamepad2.dpad_left) {//brake
                    intakePressed = 0;
                    intakeServoR.setPower(0);
                    intakeServoL.setPower(0);
                    idle();
                }
                else if (gamepad2.dpad_right) { //intake
                    intakePressed = 1;
                    intakeServoR.setPower(COUNTER_CLOCKWISE_POWER);
                    intakeServoL.setPower(CLOCKWISE_POWER);
                    idle();
                }
            } else if (gamepad2.dpad_right) { //braking & intake
                intakePressed = 1;
                intakeServoR.setPower(COUNTER_CLOCKWISE_POWER);
                intakeServoL.setPower(CLOCKWISE_POWER);
                idle();
            } else if (gamepad2.dpad_left) { //braking & outtake
                intakePressed = 2;
                intakeServoR.setPower(CLOCKWISE_POWER);
                intakeServoL.setPower(COUNTER_CLOCKWISE_POWER);
                idle();
            }
            telemetry.addData("Intake Status=",intakePressed);

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
            slideExtendR = extendServoR.getPosition();
            if ((gamepad2.right_trigger > TRIGGER_THRESHOLD) && (slideExtendR < MISUMI_EXTEND_LIMIT_R)) {  // (armPosition < 0)
                slideExtendR = slideExtendR + MISUMI_STEP_RATIO_1 * (gamepad2.right_trigger*MISUMI_STEP_RATIO_2);
                if (slideExtendR>MISUMI_EXTEND_LIMIT_R) slideExtendR=MISUMI_EXTEND_LIMIT_R;
                extendServoR.setPosition(slideExtendR);
                extendServoL.setPosition(-slideExtendR + 1);
            } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD && (slideExtendR > MISUMI_RETRACT_LIMIT_R)) {
                slideExtendR = slideExtendR - MISUMI_STEP_RATIO_1 * (gamepad2.left_trigger*MISUMI_STEP_RATIO_2);
                if (slideExtendR<MISUMI_RETRACT_LIMIT_R) slideExtendR=0;
                extendServoR.setPosition(slideExtendR);
                extendServoL.setPosition(-slideExtendR + 1);
            }
            if (debug_mode) {
                telemetry.addData("slideExtendR:", slideExtendR);
                telemetry.addData("slideExtendL:", 1-slideExtendR);
            }


            /***************** 7. Lead Screw *****************/
            /*
            LSPositionR = LSMotorR.getCurrentPosition();
            LSPositionL = LSMotorL.getCurrentPosition();
            if (gamepad1.right_bumper && (LSPositionR <= LEAD_SCREW_UPPER_LIMIT && LSPositionL <= LEAD_SCREW_UPPER_LIMIT)) {
                LSMotorR.setPower(LEAD_SCREW_POWER);
                LSMotorL.setPower(LEAD_SCREW_POWER);
            } else if (gamepad1.left_bumper && (LSPositionR >= LEAD_SCREW_OFF_THRESHOLD && LSPositionL >= LEAD_SCREW_OFF_THRESHOLD)) {
                LSMotorR.setPower(-LEAD_SCREW_POWER);
                LSMotorL.setPower(-LEAD_SCREW_POWER);
            } else {
                LSMotorR.setPower(0);
                LSMotorL.setPower(0);
           }
             */
            if (gamepad1.back) {
                if (LSState == 0) {
                    LSMotorR.setTargetPosition(LS_ABOVE_LOWER_RUNG);
                    LSMotorL.setTargetPosition(LS_ABOVE_LOWER_RUNG);
                    LSMotorR.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorL.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSState = 1;
                } else if (LSState == 1) {
                    LSMotorR.setTargetPosition(LS_LOWER_RUNG);
                    LSMotorL.setTargetPosition(LS_LOWER_RUNG);
                    LSMotorR.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorL.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSState = 2;
                } else if (LSState == 2) {
                    LSMotorR.setTargetPosition(LEAD_SCREW_OFF_THRESHOLD);
                    LSMotorL.setTargetPosition(LEAD_SCREW_OFF_THRESHOLD);
                    LSMotorR.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorL.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSState = 0;
                }
            }

            if (debug_mode) {
                telemetry.addData("LSPositionR:", LSPositionR);
                telemetry.addData("LSPositionL:", LSPositionL);
            }
            /***************** Preset Buttons *****************/

            if (gamepad2.a){ // intake position
                slideMotor.setTargetPosition(VIPER_SLIDES_OFF_THRESHOLD);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(VIPER_SLIDES_POWER_PRESET_DOWN);

                wristServo.setPosition(0.52);
                armMotor.setTargetPosition(ARM_LOWER_LIMIT);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_PRESET);
                extendServoR.setPosition(MISUMI_RETRACT_LIMIT_R);
                extendServoL.setPosition(-MISUMI_RETRACT_LIMIT_R+1);

                intakeServoR.setPower(COUNTER_CLOCKWISE_POWER);
                intakeServoL.setPower(CLOCKWISE_POWER);
                intakePressed = 1; // intake
            }
            else if (gamepad2.b) { // outtake at high basket
                wristServo.setPosition(0.34);
                armMotor.setTargetPosition(ARM_UPPER_LIMIT);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
                extendServoR.setPosition(MISUMI_RETRACT_LIMIT_R);
                extendServoL.setPosition(-MISUMI_RETRACT_LIMIT_R+1);
                slideMotor.setTargetPosition(12000); //5000
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(VIPER_SLIDES_POWER_PRESET);
            }
            else if (gamepad2.x) { // outtake at low basket
                wristServo.setPosition(0.34);
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                armMotor.setTargetPosition(ARM_UPPER_LIMIT);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);
                extendServoR.setPosition(MISUMI_RETRACT_LIMIT_R);
                extendServoL.setPosition(-MISUMI_RETRACT_LIMIT_R+1);
                slideMotor.setTargetPosition(5000); //5000
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(VIPER_SLIDES_POWER_PRESET);
            }
            // Drivetrain Moving Position
            else if (/*gamepad1.ps && */gamepad2.ps&&(armPosition >= VIPER_SLIDES_OFF_THRESHOLD + 50)) {
                slideMotor.setTargetPosition(VIPER_SLIDES_OFF_THRESHOLD);
                slideMotor.setPower(VIPER_SLIDES_POWER_PRESET_DOWN);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //brake
                intakePressed = 0;
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);

                armMotor.setTargetPosition(ARM_UPRIGHT_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);

                extendServoR.setPosition(MISUMI_RETRACT_LIMIT_R);
                extendServoL.setPosition(-MISUMI_RETRACT_LIMIT_R+1);

                wristServo.setPosition(WRIST_DOWN);
            }
            /*
            if ((gamepad2.ps)&&(!gamepad1.ps)&&(armPosition <= VIPER_SLIDES_OFF_THRESHOLD + 50)){

                armMotor.setTargetPosition(ARM_UPRIGHT_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_POWER_TO_TARGET);

                extendServoR.setPosition(MISUMI_RETRACT_LIMIT_R);
                extendServoL.setPosition(-MISUMI_RETRACT_LIMIT_R+1);

                wristServo.setPosition(WRIST_DOWN);

                //brake
                intakePressed = 0;
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
            }
             */

            /*
            // Speciman Preset
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