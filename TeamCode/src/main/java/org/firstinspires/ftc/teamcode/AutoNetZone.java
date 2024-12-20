package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name="AutoNetZone", group="Linear Opmode")
public class AutoNetZone extends LinearOpMode {
    MecanumRobot robot = new MecanumRobot(this);

    private boolean debugMode = true;
    private ElapsedTime runtime = new ElapsedTime();
    public static int ACTION_1_TIMER = 400;
    public static int ACTION_2_TIMER = 700;
    public static int ACTION_3_TIMER = 2500;
    public static int ACTION_4_TIMER = 500;
    public static int ACTION_5_TIMER = 1500;
    public static int ACTION_6_TIMER = 500;

    FtcDashboard dashboard;
    public static int TARGET_ANGLE = -45;
    public static double TURN_POWER = 0.5;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Robot", "initializing");
        telemetry.update();

        robot.initialize();
        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Robot", "starts");
        telemetry.update();

        // ****** PID Experiment on rotation *******
/*
        robot.rotate(TARGET_ANGLE, TURN_POWER);
        sleep(3000);
        robot.rotate(TARGET_ANGLE-45, TURN_POWER);
        sleep(3000);
        robot.rotate(TARGET_ANGLE-90, TURN_POWER);
        sleep(3000);
        robot.rotate(TARGET_ANGLE-135, TURN_POWER);
        sleep(3000);
*/

        // ****** Action [1] ******

        // Rachel: If following FTC sample code RobotAutoDriveByTime
        robot.move(-1, 0, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {
            //telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            //telemetry.update();
        }

        // ****** Action [2] ******

        // Rachel: If following FTC sample code RobotAutoDriveByTime
        robot.move(0, -1, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_2_TIMER)) {
            //telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            //telemetry.update();
        }

        // ****** Action [3] ******

        robot.rotate(-45, 0.5);


        // ****** Action [4] ******

        robot.highBasket();
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_3_TIMER)) {
            //telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            //telemetry.update();
        }

        robot.outtake();
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_4_TIMER)) {
            //telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            //telemetry.update();
        }

        robot.reset();

        robot.rotate(45, 0.5);

        robot.move(0, 1, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 500));

        robot.move(-1, 0, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 2500));

        //add arm up function here

        robot.move(0, 1, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 250));
    }
}