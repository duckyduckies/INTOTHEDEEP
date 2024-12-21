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
@Autonomous(name="AutoObservationZone", group="Linear Opmode")
public class AutoObservationZone extends LinearOpMode {
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

        // ****** Action [1] ****** Assume we already have a specimen in the claw (Specimen outtake)


        robot.move(-1, 0, 0, 0.5); //left 2 tiles
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400

        robot.move(0, 1.5, 0, 0.5); //down 1 tile
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_2_TIMER)) { //700
        }
        robot.highchamberspecimenouttake();
        robot.highchamberspecimenouttake2();

        robot.move(0, -0.5, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 700));

        robot.move(0, -0.5, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 700));
        robot.reset();

        robot.move(0, -1, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400


        // ****** Action [2] ****** (Specimen outtake and intake)

        robot.rotate(90, 0.5);
        robot.clawintakefromfloor();

        robot.move(1, 0, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400

        robot.reset();
        robot.rotate(-90, 0.5);
        //Same as Action 1
        robot.move(-1, 0, 0, 0.5); //left 2 tiles
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400

        robot.move(0, 1.5, 0, 0.5); //down 1 tile
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_2_TIMER)) { //700
        }
        robot.highchamberspecimenouttake();
        robot.highchamberspecimenouttake2();

        robot.move(0, -0.5, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 700));

        robot.move(0, -0.5, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 700));
        robot.reset();

        robot.move(0, -1, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400


        // ****** Action [3] ****** (high basket)
        robot.move(2, 0, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400

        robot.move(0, 3, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400
        robot.rotate (-90, 0.5);

        robot.clawintakefromfloor();
        robot.reset();

        robot.move(0, -2, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400

        robot.move(-3, 0, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ACTION_1_TIMER)) {} //400

        robot.rotate(-45, 0.5);
        robot.highBasket();


        //robot.rotate(45, 0.5);

        /*robot.move(0, 1, 0, 0.5); // goes forward for 1 tile
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 700));

        //robot.move(1, 0, 0, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 500)); // sideways for 1 tile (right)
        //add arm up function here
        */
    }
}