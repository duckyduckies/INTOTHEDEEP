package org.firstinspires.ftc.teamcode;

// import

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
@Disabled
public class AutoTemplate extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode CONSTANTs. */

    /*
    * Please name each constant by the actual action
    * because it's very possible that we will insert or delete actions
     */
    static final double ACTION_1_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_2_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_3_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_4_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_5_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_6_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_7_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_8_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_9_INPUT_PARAMETER_1 = 1.0;
    static final double ACTION_10_INPUT_PARAMETER_1 = 1.0;
    static final double OP_CONSTANT_2 = -1.0;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Configure the drive system variables.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // ****** Action [1] Method ******
        // Description: Move to the right with 1 small step (t1)
        // Input parameter: ACTION_1_INPUT_PARAMETER_1
        moveRight(ACTION_1_INPUT_PARAMETER_1);
        /*
         * Rachel: I prefer using the existing move function and input the direction & moving amounts
         *          And maybe another hold function and input the time
        */

        // ****** Action [2] Method ******
        // Description: Move forward 2 normal steps (t2)
        // Input parameter: ACTION_2_INPUT_PARAMETER_1
        moveForward(ACTION_2_INPUT_PARAMETER_1);
        /*
        * Rachel: I prefer using the existing move function and input the direction & moving amounts
        *           And maybe another hold function and input the time
        */


        // ****** Action [3] Method ******
        // Description: Turn 45 degrees counterclockwise (t3)
        // Input parameter: ACTION_3_INPUT_PARAMETER_1
        turn45DegreesCounterclockwise(ACTION_3_INPUT_PARAMETER_1);
        /*
         * Rachel: I prefer a general rotate function and input an angle
         */


        /*
         * Rachel: Action 4 & 5 will be combined into the one "out take preset" function
         *          which will be shared with TeleOp
         */
        // ****** Action [4] Method ******
        // Description: Extend viper slides (target 1)
        // Input parameter: ACTION_4_INPUT_PARAMETER_1
        extendViperSlides(ACTION_4_INPUT_PARAMETER_1);


        // ****** Action [5] Method ******
        // Description: Arm down wrist up (target 2/position 1)
        // Input parameter: ACTION_5_INPUT_PARAMETER_1
        armDownWristUp(ACTION_5_INPUT_PARAMETER_1);


        // ****** Action [6] Method ******
        // Description: Out take sample
        // Input parameter: ACTION_6_INPUT_PARAMETER_1
        outTakeSample(ACTION_6_INPUT_PARAMETER_1);



        // ****** Action [7] Method ******
        // Description: Turn 45 degrees clockwise (t5)
        // Input parameter: ACTION_7_INPUT_PARAMETER_1
        turn45DegreesClockwise(ACTION_7_INPUT_PARAMETER_1);
        /*
         * Rachel: I prefer a general rotate function and input an angle
         */


        // ****** Action [8] Method ******
        // Description: Move to the right 3 normal steps (t6)
        // Input parameter: ACTION_8_INPUT_PARAMETER_1
        moveRight(ACTION_8_INPUT_PARAMETER_1);
        /*
         * Rachel: I prefer using the existing move function and input the direction & moving amounts
         *           And maybe another hold function and input the time
         */



        // ****** Action [9] Method ******
        // Description:  Move backward 2 normal steps (t7)
        // Input parameter: ACTION_9_INPUT_PARAMETER_1
        moveBackward(ACTION_9_INPUT_PARAMETER_1);
        /*
         * Rachel: I prefer using the existing move function and input the direction & moving amounts
         *           And maybe another hold function and input the time
         */


        // ****** Action [10] Method ******
        // Description: Extend lead screw to touch ascent 1 (target 3)
        // Input parameter: ACTION_10_INPUT_PARAMETER_1
        extendLeadScrew(ACTION_10_INPUT_PARAMETER_1);
        /*
         * Rachel: This can be named as ascend 1 preset and be shared with TeleOp
         */
    }

    public void moveRight(double inputParameter1) {
        System.out.println("Method: moveRight()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);

    }

    public void moveForward(double inputParameter1) {
        System.out.println("Method: moveForward()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);

    }

    public void turn45DegreesCounterclockwise(double inputParameter1) {
        System.out.println("Method: turn45DegreesCounterclockwise()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);
}


    public void extendViperSlides(double inputParameter1) {
        System.out.println("Method: extendViperSlides()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);

    }


    public void armDownWristUp(double inputParameter1) {
        System.out.println("Method: armDownWristUp()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);

    }


    public void outTakeSample(double inputParameter1) {
        System.out.println("Method: outTakeSample()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);

        }


        public void turn45DegreesClockwise(double inputParameter1) {
            System.out.println("Method: turn45DegreesClockwise()");
            System.out.println("Input parameter: inputParameter1 = " + inputParameter1);

        }


    public void moveBackward(double inputParameter1) {
        System.out.println("Method: moveBackward()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);

    }



    public void extendLeadScrew(double inputParameter1) {
        System.out.println("Method: extendLeadScrew()");
        System.out.println("Input parameter: inputParameter1 = " + inputParameter1);
    }