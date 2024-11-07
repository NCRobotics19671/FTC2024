//   ******Front******
//   * 0           1 *
//   *               *
//   *               *
//   *               *
//   * 2           3 *
//   ******Back*******

// Motor 0 --- Pin 0 --- motorFrontLeft
// Motor 1 --- Pin 1 --- motorFrontRight
// Motor 2 --- Pin 2 --- motorBackLeft
// Motor 3 --- Pin 3 --- motorBackRight

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(group="Iterative Opmode")

public class Test2 extends OpMode {
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor Arm = null;
    private DcMotor Alien = null;
    private DcMotor PickupL = null;
    private DcMotor PickupR = null;
    private Servo RightServo = null;
    private Servo LeftServo = null;

    TouchSensor touch;


    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;


    double powerCoef = 0.75;
    float left = 0;

    float right = 0;
    boolean toggleA = true;

    boolean toggleB = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();


        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry




        /*double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x*1.1; //* 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = -(rotY + rotX + rx) / denominator;
        double backLeftPower = -(rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        motorFrontLeft.setPower(0.95*powerCoef*frontLeftPower);
        motorBackLeft.setPower(powerCoef*backLeftPower);
        motorFrontRight.setPower(powerCoef*frontRightPower);
        motorBackRight.setPower(powerCoef*backRightPower);*/


        /**********Pickup********/
        //if (!gamepad1.a) {
        //    toggleA = true;
        //}

        if (!gamepad1.a) {
            toggleA = true;
        }

        if(gamepad1.a && toggleA){
            toggleA=false;
            if(!toggleB){
                toggleB = true;
                LeftServo.setPosition(0.5);
                RightServo.setPosition(1);
            }
            else if(toggleB){
                toggleB = false;
                LeftServo.setPosition(0.3);
                RightServo.setPosition(0);
            }

        }



        /********End Pickup********/


        //Left trigger is squeezed but right trigger is not so we are going down


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Yaw (Z)", "%.2f Rad. (Heading)", botHeading);
        telemetry.addData("A", toggleB);

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }


}
