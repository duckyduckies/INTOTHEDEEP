package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor LSMotorR = hardwareMap.dcMotor.get("LSMotorR");
        DcMotor LSMotorL = hardwareMap.dcMotor.get("LSMotorL");
        DcMotor SlideMotorR = hardwareMap.dcMotor.get("RightSlideMotor");
        DcMotor SlideMotorL = hardwareMap.dcMotor.get("LeftSlideMotor");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        double LSPositionR = LSMotorR.getCurrentPosition();
        double LSPositionL = LSMotorL.getCurrentPosition();
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

//            double speedReduction = 1;


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

            telemetry.addData("frontLeftPower_mod: ", frontLeftPower);
            telemetry.addData("frontLeftPower_mod: ", backLeftPower);
            telemetry.addData("frontLeftPower_mod: ", frontRightPower);
            telemetry.addData("frontLeftPower_mod: ", backRightPower);
            telemetry.addData("frontLeftPower_mod: ", frontLeftPower_mod);
            telemetry.addData("frontLeftPower_mod: ", backLeftPower_mod);
            telemetry.addData("frontLeftPower_mod: ", frontRightPower_mod);
            telemetry.addData("frontLeftPower_mod: ", backRightPower_mod);

            if(gamepad1.right_bumper){
                LSMotorR.setPower(0.2);
                LSMotorL.setPower(0.2);
            }
            else if (gamepad1.left_bumper){
                LSMotorR.setPower(-0.2);
                LSMotorL.setPower(-0.2);
            }
            else {
                LSMotorR.setPower(0);
                LSMotorL.setPower(0);
            }
            if(gamepad2.left_stick_y<0){
                SlideMotorR.setPower(-gamepad2.left_stick_y);
                SlideMotorL.setPower(-gamepad2.left_stick_y);
            }
            else if (gamepad2.left_stick_y>0){
                SlideMotorR.setPower(-gamepad2.left_stick_y);
                SlideMotorL.setPower(-gamepad2.left_stick_y);
            }
            else {
                SlideMotorR.setPower(0);
                SlideMotorL.setPower(0);
            }

            telemetry.update();
        }
    }

    private double DcMotorPowerModifier(double Power) {
        return Math.pow(Math.tanh(Power)/Math.tanh(1),3);
    }

    private double DcMotorPowerModifierAdv(double Power, int eqVer) {
        if (eqVer == 1) {
            return Math.pow(Math.tanh(Power)/Math.tanh(1),3);
        } else if (eqVer == 2) {
            return Math.pow(Power, 3);
        } else if (eqVer == 3) {
            return Math.pow(Power, 5);
        } else {
            return Power;
        }
    }
}