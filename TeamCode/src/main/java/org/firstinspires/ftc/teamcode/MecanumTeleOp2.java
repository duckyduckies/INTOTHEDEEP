package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

@TeleOp
@Config
public class MecanumTeleOp2 extends LinearOpMode {
    MecanumRobot robot = new MecanumRobot(this);
    private ElapsedTime runtime = new ElapsedTime();
    private boolean debugMode = true;
    private boolean leadScrewDebug = false;
    private boolean touchpadCurrState = true;
    private boolean touchpadPrevState = true;
    private boolean back2PrevState = true;
    private boolean back2CurrState = true;
    private boolean back1PrevState = false;
    private boolean back1CurrState = false;
    private boolean outtakePrevState = false;
    private boolean outtakeCurrState = false;
    private boolean intakePrevState = false;
    private boolean intakeCurrState = false;
    private boolean LSPrevState = false;
    private boolean LSCurrState = false;
    private final static double TRIGGER_THRESHOLD = 0.25;
    public static boolean TEAM_COLOR_RED = true; // 224, 18, 76, 154

    /*
    //***************** 0. IMU *****************
    IMU imu;

    //***************** 1. Mecanum Drivetrain *****************
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

    //***************** 2. Viper Slides *****************
    //28 inch,
    //private final static double VIPER_SLIDES_POWER = 0.75;
    private final static double VIPER_SLIDES_POWER_MANUAL = 1;
    public static double VIPER_SLIDES_POWER_PRESET = 1;
    public static double VIPER_SLIDES_POWER_PRESET_DOWN = 0.8;
    private final static int VIPER_SLIDES_INITIAL_POSITION = 0;
    private final static int VIPER_SLIDES_STEP = 200;
    private final static int VIPER_SLIDES_UPPER_LIMIT = 5000;
    private final static int VIPER_SLIDES_LOWER_LIMIT = -100;
    private final static double VIPER_SLIDES_CALIBRATE_DISTANCE = 3.0;
    DcMotor slideMotorR;
    DcMotor slideMotorL;
    int slidePositionR = VIPER_SLIDES_INITIAL_POSITION;
    int slidePositionL = VIPER_SLIDES_INITIAL_POSITION;
    //***************** 3. Arm *****************

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
    //***************** 4. Wrist *****************
    private final static double WRIST_STEP = 0.02;
    private final static double WRIST_UP = 0.38;
    private final static double WRIST_DOWN = 1;
    double wristPosition = WRIST_DOWN; // default position is down
    Servo wristServo;

    //***************** 5. Claw Intake *****************
    private final static int CLAW_INITIAL_STATE = 0;
    private final static double CLOCKWISE_POWER = -1.0;
    private final static double COUNTER_CLOCKWISE_POWER = 1.0;
    private final static double SAMPLE_IN_DISTANCE = 2.5;
    // Intake state:
    // 0: NotOutake
    int clawState = CLAW_INITIAL_STATE;
    //int intakedirection = 0;
    CRServo clawServoR;
    CRServo clawServoL;

    //***************** 6. MiSUMi Slides *****************
    private final static double MISUMI_STEP_RATIO_1 = 0.1; // CY defines it
    private final static double MISUMI_EXTEND_LIMIT_R = 0.35;
    private final static double MISUMI_RETRACT_LIMIT_R = 0;
    private final static double MISUMI_STEP_RATIO_2 = MISUMI_EXTEND_LIMIT_R/1; // CC defines it
    Servo extendServoR;
    Servo extendServoL;
    double slideExtendR = MISUMI_RETRACT_LIMIT_R;

    //***************** 7. Lead Screw *****************
    private final static double LEAD_SCREW_POWER = 0.7;
    private final static double LEAD_SCREW_POWER_PRESET = 1;
    private final static int LEAD_SCREW_INITIAL_POSITION = 0;
    //private final static int LEAD_SCREW_STEP = 150;
    private final static int LEAD_SCREW_UPPER_LIMIT = 34000;
    private final static int LEAD_SCREW_LOWER_LIMIT = 0;
    private final static int LEAD_SCREW_OFF_THRESHOLD = 300;

    DcMotor LSMotorR;
    //DcMotor LSMotorL = hardwareMap.dcMotor.get("LSMotorL");
    int LSPositionL = LEAD_SCREW_INITIAL_POSITION;
    int LSPositionR = LEAD_SCREW_INITIAL_POSITION;
    int LSState = 0;
    //***************** 8. Color Sensor *****************
    private final static int BLUE_HUE = 224;
    private final static int RED_HUE = 18;
    private final static int YELLOW_HUE = 76;
    private final static int NO_SAMPLE_HUE = 154;
    ColorSensor colorSensor;
    ColorSensor viperSlidesSensor;
    //***************** 9. LED *****************
    RevBlinkinLedDriver blinkinLedDriver;
    private final static RevBlinkinLedDriver.BlinkinPattern DEFAULT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GRAY;
    private final static RevBlinkinLedDriver.BlinkinPattern YELLOW_PATTERN = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private final static RevBlinkinLedDriver.BlinkinPattern RED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;
    private final static RevBlinkinLedDriver.BlinkinPattern BLUE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    protected enum DisplayKind {
        MANUAL,
        AUTO
    }
    private DisplayKind displayKind;
    private String currentPattern;

    public String getBlinkinLEDDisplayPattern() {
        return currentPattern;
    }
    protected void setBlinkinLEDDisplayPattern(RevBlinkinLedDriver.BlinkinPattern pattern)
    {
        blinkinLedDriver.setPattern(pattern);
        currentPattern = pattern.toString();
    }
    //***************** 10. FTC Dashboard *****************
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
    //***************** Preset Buttons *****************
    public static double INTAKE_PRESET_WRIST_POS = 0.5;
    public static int INTAKE_PRESET_ARM_POS = -1400;
    public static int OUTTAKE_PRESET_HIGH_BASKET_VIPER_POS = 4200;
    public static int OUTTAKE_PRESET_LOW_BASKET_VIPER_POS = 1800;
    public static double OUTTAKE_PRESET_WRIST_POS = 0.36;
    public static int OUTTAKE_PRESET_ARM_POS = 350;
    public static double OUTTAKE_PRESET_CHAMBER_WRIST_POS = 0.5;
    public static double OUTTAKE_PRESET_MISUMI_R = 0.1;
    public static int OUTTAKE_PRESET_HIGH_CHAMBER_VIPER_1 = 2000;
    public static int OUTTAKE_PRESET_HIGH_CHAMBER_VIPER_2 = 1450;
    public static int LS_ABOVE_LOWER_RUNG = 34000;
    public static int VIPER_SLIDE_DOWN_TIMER = 1500;
    public static int VIPER_SLIDE_UP_TIMER = 500;
    */
    private class DriveThread extends Thread
    {
        public DriveThread()
        {
            this.setName("DriveThread");
        }
        /*
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

        }
         */
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
                    x = gamepad1.left_stick_x; // Remember, Y stick value is reversed
                    y = -gamepad1.left_stick_y; // Counteract imperfect strafing
                    rx = gamepad1.right_stick_x;

                    robot.move(x,y,rx,1);

                    if (gamepad1.dpad_up) {
                        robot.move(0, 1,0,MecanumRobot.DPAD_FORWARD_BACKWARD_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_down) {
                        robot.move(0, -1,0,MecanumRobot.DPAD_FORWARD_BACKWARD_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_right) {
                        robot.move(1, 0,0,MecanumRobot.DPAD_SIDEWAY_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_left) {
                        robot.move(-1, 0,0,MecanumRobot.DPAD_SIDEWAY_POWER_RATIO);
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
    /*
    private class DriveThreadFieldCentric extends Thread
    {
        public DriveThreadFieldCentric()
        {
            this.setName("DriveThread");
        }

        public void move(double x, double y, double rx, double botHeading, double powerScale) {
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

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            try
            {
                double x,y,rx,botHeading;
                while (opModeIsActive() && !isInterrupted())
                {

                    x = gamepad1.left_stick_x; // Remember, Y stick value is reversed
                    y = -gamepad1.left_stick_y; // Counteract imperfect strafing
                    rx = gamepad1.right_stick_x;

                    // This button choice was made so that it is hard to hit on accident,
                    // it can be freely changed based on preference.
                    // The equivalent button is start on Xbox-style controllers.
                    // if (gamepad1.options) {
                    //    imu.resetYaw();
                    //}
                    
                    botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    move(x,y,rx,botHeading,1);

                    if (gamepad1.dpad_up) {
                        move(0, 1,0,botHeading,DPAD_FORWARD_BACKWARD_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_down) {
                        move(0, -1,0,botHeading,DPAD_FORWARD_BACKWARD_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_right) {
                        move(1, 0,0,botHeading,DPAD_SIDEWAY_POWER_RATIO);
                    }
                    else if (gamepad1.dpad_left) {
                        move(-1, 0,0,botHeading,DPAD_SIDEWAY_POWER_RATIO);
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
    */
    @Override
    public void runOpMode() throws InterruptedException {
        // CONFIGURATION--------------------------------------------------------------

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.initializeBeforeStart();

        waitForStart();

        telemetry.addData("Robot", "Starting and initializing");
        telemetry.update();

        robot.initialize();

        Thread  driveThread;
        //if (robotCentricDrive) {
        driveThread = new DriveThread();
        //} //else {
        //    driveThread = new DriveThreadFieldCentric();
        //}
        driveThread.start();

        if (isStopRequested()) return;

        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

        telemetry.addData("Robot", "finishes initialization");
        telemetry.update();

//OP MODE CODE-------------------------------------------------------------------------
        while (opModeIsActive()) {
            //logGamepad(telemetry, gamepad1, "gamepad1");
            //logGamepad(telemetry, gamepad2, "gamepad2");

            //if (robotCentricDrive)
                //telemetry.addData("Robot centric driving",1);
            //else
            //    telemetry.addData("Field centric driving",0);

            /***************** Toggle Button Status *****************/

            // Click "back" button to toggle the debug view

            // check the status of the back button on gamepad2.
            back2CurrState = gamepad2.back;
            // check for button state transitions.
            if (back2CurrState && !back2PrevState)  {
                // button is transitioning to a pressed state. So Toggle debug mode
                debugMode = !debugMode;
            }
            // update previous state variable.
            back2PrevState = back2CurrState;
            if (debugMode) {
                telemetry.addData("carlos loves katelyn",true);
            }
            else {
                telemetry.addData("carlos loves katelyn", false);
            }

            touchpadCurrState = gamepad1.touchpad;
            // check for button state transitions.
            if (touchpadCurrState && !touchpadPrevState)  {
                // button is transitioning to a pressed state. So Toggle debug mode
                TEAM_COLOR_RED = !TEAM_COLOR_RED;
            }
            // update previous state variable.
            touchpadPrevState = touchpadCurrState;
            if (TEAM_COLOR_RED) {
                telemetry.addData("team color red is ",true);
            }
            else {
                telemetry.addData("team color blue is ", true);
            }

            back1CurrState = gamepad1.back;
            if (back1CurrState && !back1PrevState) {
                leadScrewDebug = !leadScrewDebug;
                if (leadScrewDebug) {
                    robot.LSMotorR.setPower(0);
                    //LSMotorL.setPower(0);
                    robot.LSMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //LSMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
            back1PrevState = back1CurrState;
            if (leadScrewDebug) {
                telemetry.addData("Lead Screw Debug Mode enabled", 1);
            }

            back1CurrState = gamepad1.back;
            if (back1CurrState && !back1PrevState) {
                leadScrewDebug = !leadScrewDebug;
                if (leadScrewDebug) {
                    robot.LSMotorR.setPower(0);
                    //LSMotorL.setPower(0);
                    robot.LSMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //LSMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
            back1PrevState = back1CurrState;
            if (leadScrewDebug) {
                telemetry.addData("Lead Screw Debug Mode enabled", 1);
            }
            /***************** 8. Color Sensor *****************/
            robot.autoOuttakeBasedOnColor(telemetry);

            /***************** 1. Mecanum Drivetrain *****************/

            //moved to move function

            /***************** 2. Viper Slides *****************/
            int slidePositionR = robot.slideMotorR.getCurrentPosition();
            int slidePositionL = robot.slideMotorL.getCurrentPosition();
            // first priority
            if (gamepad2.left_stick_y < 0 && slidePositionR <= MecanumRobot.VIPER_SLIDES_UPPER_LIMIT) { // slides go up
                robot.viperSlidesUp(telemetry);
            } else if (gamepad2.left_stick_y > 0 && slidePositionR > MecanumRobot.VIPER_SLIDES_LOWER_LIMIT) {
                robot.viperSlidesDown(telemetry);
            }

            if (debugMode) {
                telemetry.addData("slidePositionR: ", slidePositionR);
                telemetry.addData("slidePositionL: ", slidePositionL);
            }
            /***************** 3. Arm *****************/
            //armMotor.setPower(-gamepad2.right_stick_y/4);
            int armPosition=robot.armMotor.getCurrentPosition();
            if ((gamepad2.right_stick_y < 0)  &&
            ((slidePositionR<MecanumRobot.OUTTAKE_PRESET_HIGH_CHAMBER_VIPER_1 && armPosition <= MecanumRobot.ARM_INITIAL_POSITION)
            ||(slidePositionR>=MecanumRobot.OUTTAKE_PRESET_HIGH_CHAMBER_VIPER_1 && armPosition <= MecanumRobot.ARM_UPPER_LIMIT))) { // joystick above the origin; arm raises up
                robot.armUp();
            } else if (gamepad2.right_stick_y > 0 && armPosition >= MecanumRobot.ARM_LOWER_LIMIT) { // joystick below the origin; arm puts down
                robot.armDown();
            } //else {
                //armMotor.setPower(0); // RUN_WITHOUT_ENCODER mode
            //}
            if (debugMode) {
                telemetry.addData("armPosition:", armPosition);
            }

            /***************** 4. Wrist *****************/
            double wristPosition = robot.wristServo.getPosition();
            if (gamepad2.dpad_down) {
                robot.wristDown();
            }
            else if (gamepad2.dpad_up) {
                robot.wristUp();
            }
            if (debugMode) {
                telemetry.addData("wristPosition:", wristPosition);
            }

            /***************** 5. Claw Intake *****************/
            // Intake state:
            // 0: Not rotating
            // 1: Intake
            // 2: Outake
            outtakeCurrState = gamepad2.left_bumper;
            intakeCurrState = gamepad2.right_bumper;
            if (intakeCurrState && !intakePrevState) {
                if (robot.clawState == 1) {
                    robot.clawState = 0;
                }
                else {
                    robot.clawState = 1;
                }
            }
            intakePrevState = intakeCurrState;
            if (outtakeCurrState && !outtakePrevState) {
                if (robot.clawState == 2) {
                    robot.clawState = 0;
                }
                else {
                    robot.clawState = 2;
                }
            }
            outtakePrevState = outtakeCurrState;

            robot.setClawState(telemetry);

            /*
            if (clawState == 1) { //intaking
                if (gamepad2.right_bumper) { //brake
                    clawState = 0;
                    clawServoR.setPower(0);
                    clawServoL.setPower(0);
                    idle();
                }
                else if (gamepad2.left_bumper){ //outtake
                    clawState = 2;
                    clawServoR.setPower(CLOCKWISE_POWER);
                    clawServoL.setPower(COUNTER_CLOCKWISE_POWER);
                    idle();
                }
            } else if (clawState == 2) { //outtaking
                if (gamepad2.left_bumper) {//brake
                    clawState = 0;
                    clawServoR.setPower(0);
                    clawServoL.setPower(0);
                    idle();
                }
                else if (gamepad2.right_bumper) { //intake
                    clawState = 1;
                    clawServoR.setPower(COUNTER_CLOCKWISE_POWER);
                    clawServoL.setPower(CLOCKWISE_POWER);
                    idle();
                }
            } else if (gamepad2.right_bumper) { //braking & intake
                clawState = 1;
                clawServoR.setPower(COUNTER_CLOCKWISE_POWER);
                clawServoL.setPower(CLOCKWISE_POWER);
                idle();
            } else if (gamepad2.left_bumper) { //braking & outtake
                clawState = 2;
                clawServoR.setPower(CLOCKWISE_POWER);
                clawServoL.setPower(COUNTER_CLOCKWISE_POWER);
                idle();
            }
            telemetry.addData("Intake Status=",clawState);


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
                clawServoR.setPower(-1.0);
                clawServoL.setPower(1.0);
            } else if (intakedirection == -1){
                clawServoR.setPower(1.0);
                clawServoL.setPower(-1.0);
            } else {
                clawServoR.setPower(0);
                clawServoL.setPower(0);
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
            double slideExtendR = robot.extendServoR.getPosition();
            if ((gamepad2.right_trigger > TRIGGER_THRESHOLD) && (slideExtendR < MecanumRobot.MISUMI_EXTEND_LIMIT_R) && (armPosition <= MecanumRobot.ARM_UPPER_LIMIT)) {  // (armPosition < 0)
                robot.extendHorizontalSlides(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD && (slideExtendR > MecanumRobot.MISUMI_RETRACT_LIMIT_R)) {
                robot.retractHorizontalSlides(gamepad2.left_trigger);
            }
            if (debugMode) {
                telemetry.addData("slideExtendR:", slideExtendR);
                telemetry.addData("slideExtendL:", 1-slideExtendR);
            }


            /***************** 7. Lead Screw ****************

            LSPositionR = LSMotorR.getCurrentPosition();
            //LSPositionL = LSMotorL.getCurrentPosition();
            if (leadScrewDebug) {
                if (gamepad1.right_bumper && (LSPositionR <= LEAD_SCREW_UPPER_LIMIT)) {
                    LSMotorR.setPower(LEAD_SCREW_POWER);
                    //LSMotorL.setPower(LEAD_SCREW_POWER);
                } else if (gamepad1.left_bumper) {// && (LSPositionR >= LEAD_SCREW_OFF_THRESHOLD && LSPositionL >= LEAD_SCREW_OFF_THRESHOLD)) {
                    LSMotorR.setPower(-LEAD_SCREW_POWER);
                    //LSMotorL.setPower(-LEAD_SCREW_POWER);
                } else {
                    LSMotorR.setPower(0);
                    //LSMotorL.setPower(0);
                }
            }
            else {
                LSCurrState = gamepad2.ps;
                if (LSCurrState && !LSPrevState) {
                    if (LSState == 0) {
                        LSState = 1; // above rung
                    } else if (LSState == 1) {
                        LSState = 2; // below rung
                    } else if (LSState == 2) {
                        LSState = 3; // above rung
                    } else if (LSState == 3) {
                        LSState = 4; // default position
                    } else if (LSState == 4) {
                        LSState = 5; // lose power
                    } else if (LSState == 5) {
                        LSState = 1; // above rung
                    }
                }
                LSPrevState = LSCurrState;
                if (LSState == 5) {// lose power
                    LSMotorR.setPower(0);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else if (LSState == 4) {//default position
                    LSMotorR.setTargetPosition(LEAD_SCREW_OFF_THRESHOLD);
                    //LSMotorL.setTargetPosition(LEAD_SCREW_OFF_THRESHOLD);
                    LSMotorR.setPower(LEAD_SCREW_POWER_PRESET);
                    //LSMotorL.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //LSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (LSState == 1) {//above bar
                    LSMotorR.setTargetPosition(LS_ABOVE_LOWER_RUNG);
                    //LSMotorL.setTargetPosition(LS_ABOVE_LOWER_RUNG);
                    LSMotorR.setPower(LEAD_SCREW_POWER_PRESET);
                    //LSMotorL.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //LSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (LSState == 2) {//below bar
                    LSMotorR.setTargetPosition(LEAD_SCREW_OFF_THRESHOLD);
                    //LSMotorL.setTargetPosition(LS_LOWER_RUNG);
                    LSMotorR.setPower(LEAD_SCREW_POWER_PRESET);
                    //LSMotorL.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //LSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (LSState == 3) {//above bar
                    LSMotorR.setTargetPosition(LS_ABOVE_LOWER_RUNG);
                    //LSMotorL.setTargetPosition(LS_ABOVE_LOWER_RUNG);
                    LSMotorR.setPower(LEAD_SCREW_POWER_PRESET);
                    //LSMotorL.setPower(LEAD_SCREW_POWER_PRESET);
                    LSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //LSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            if (debugMode) {
                telemetry.addData("LSPositionR:", LSPositionR);
                telemetry.addData("LSPositionL:", LSPositionL);
                telemetry.addData("Lead Screw State: ", LSState);
            }
             */

            /***************** Preset Buttons *****************/

            if (gamepad2.b){ // intake position
                robot.clawintakefromfloor();
            }

            if (gamepad2.y || gamepad2.a) { // outtake at high or low basket
                if (gamepad2.y) robot.outtakeBasket(MecanumRobot.HIGH_BASKET);
                else if (gamepad2.a) robot.outtakeBasket(MecanumRobot.LOW_BASKET);
            }
            /*
            if (gamepad2.dpad_right) { //specimen outtake

                robot.highchamberspecimenouttake();
            }
            if (gamepad2.dpad_left) {
                robot.highchamberspecimenouttake2();
            }
            */

            // Drivetrain Moving Position
            if (gamepad2.x) {
                robot.reset();
            }

            if (debugMode && robot.viperSlidesSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", 
                        ((DistanceSensor) robot.viperSlidesSensor).getDistance(DistanceUnit.CM));
            }


            telemetry.update();
            sleep(20);
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