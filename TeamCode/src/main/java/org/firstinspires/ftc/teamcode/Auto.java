package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="RedAuto", group="Linear Opmode")
public class Auto extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor armMotor = null;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
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