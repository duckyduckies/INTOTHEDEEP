package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoNetZone", group="Linear Opmode")
public class AutoNetZone extends LinearOpMode {

    boolean debug_mode = true;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private ElapsedTime runtime = new ElapsedTime();

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double y = -0;
        double x = 0.3 * 1.1;
        double rx = 0;

        frontLeftPower = ((y + x + rx));
        backLeftPower = ((y - x + rx));
        frontRightPower = ((y - x - rx));
        backRightPower = ((y + x - rx));
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        try {
            Thread.sleep(700); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        try {
            Thread.sleep(200); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        y = -0;
        x = 0 * 1.1;
        rx = -0.5;

        frontLeftPower = ((y + x + rx));
        backLeftPower = ((y - x + rx));
        frontRightPower = ((y - x - rx));
        backRightPower = ((y + x - rx));
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        try {
            Thread.sleep(500); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        y = -0.5;
        x = 0 * 1.1;
        rx = 0;

        frontLeftPower = ((y + x + rx));
        backLeftPower = ((y - x + rx));
        frontRightPower = ((y - x - rx));
        backRightPower = ((y + x - rx));
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

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
        y = -0;
        x = 0 * 1.1;
        rx = -0.5;

        frontLeftPower = ((y + x + rx));
        backLeftPower = ((y - x + rx));
        frontRightPower = ((y - x - rx));
        backRightPower = ((y + x - rx));
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        try {
            Thread.sleep(500); // Sleep for 1 second (1000 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        y = -1;
        x = 0 * 1.1;
        rx = 0;

        frontLeftPower = ((y + x + rx));
        backLeftPower = ((y - x + rx));
        frontRightPower = ((y - x - rx));
        backRightPower = ((y + x - rx));
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        try {
            Thread.sleep(500); // Sleep for 0.5 second (500 milliseconds)
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}