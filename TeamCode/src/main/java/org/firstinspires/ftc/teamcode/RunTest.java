//   ******Front******
//   * 0           2 *
//   *               *
//   *               *
//   *               *
//   * 1           3 *
//   ******Back*******

// Motor 0 --- Pin 0 --- motorFrontLeft
// Motor 1 --- Pin 1 --- motorBackLeft
// Motor 2 --- Pin 2 --- motorFrontRight
// Motor 3 --- Pin 3 --- motorBackRight

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

@TeleOp(name="Runtest", group="Iterative Opmode")

public class RunTest extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorBackRight = null;
    private DcMotor Arm = null;
    private DcMotor Alien = null;

    private Servo Claw = null;


    TouchSensor touch;


    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
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

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 3; // Left
        usbFacingDirectionPosition = 0; // Up

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;

       

        // Declare our motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");


        Arm = hardwareMap.dcMotor.get("Arm");
        Alien = hardwareMap.dcMotor.get("Alien");
        Claw = hardwareMap.get(Servo.class, "Claw");

        touch = hardwareMap.get(TouchSensor.class, "Touch");



        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Alien.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests



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
        if (touch.isPressed())
        {Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Alien.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Alien.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry

        if(gamepad1.a){motorFrontRight.setPower(1);
        }else{motorFrontRight.setPower(0);}
        if(gamepad1.b){motorFrontLeft.setPower(1);}else{motorFrontLeft.setPower(0);}
        if(gamepad1.x){motorBackLeft.setPower(1);}else{motorBackLeft.setPower(0);}
        if(gamepad1.y){motorBackRight.setPower(1);}else{motorBackRight.setPower(0);}
        /*double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x*1.1; //* 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        */

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double botHeading = orientation.getYaw(AngleUnit.RADIANS);

        /*
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


        if (gamepad1.right_bumper && !touch.isPressed() && Alien.getCurrentPosition() < 7690) {
            Alien.setPower(1);


        }
        else {
            if (gamepad1.left_bumper && Alien.getCurrentPosition() > 0) {
                Alien.setPower(-1);


            }
            else {
                Alien.setPower(0);

            }

        }

        if (!gamepad1.a) {
            toggleA = true;
        }

        if(gamepad1.a && toggleA){
            toggleA=false;
            if(!toggleB){
                toggleB = true;

                Claw.setPosition(1);
            }
            else if(toggleB){
                toggleB = false;

                Claw.setPosition(0);
            }

        }




        if (Arm.getCurrentPosition() > 4200)
        {right = 0;}
        else {right = gamepad1.right_trigger;}
        if (!touch.isPressed())
        {left = gamepad1.left_trigger;}
        else {left = 0;}
        float power = right - left;

        Arm.setPower(power);

        /**********Pickup********/
    /*    if (!gamepad1.a) {
            toggleA = true;
        }

        if(gamepad1.a && toggleA){
            toggleA=false;
            if(PickupL.getPower() != 0){
                PickupL.setPower(0);
                PickupR.setPower(0);

            }
            else if(PickupL.getPower() == 0){
                PickupL.setPower(1);
                PickupR.setPower(1);

            }

        }

        if (!gamepad1.b) {
            toggleB = true;
        }

        if(gamepad1.b && toggleB){
            toggleB=false;
            if(PickupL.getPower() != 0){
                PickupL.setPower(0);
                PickupR.setPower(0);


            }
            else if(PickupL.getPower() == 0){
                PickupL.setPower(-1);
                PickupR.setPower(-1);

            }

        }
*/
        /********End Pickup********/

        telemetry.addData("Alien", Alien.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
        telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
        telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
        telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());

        telemetry.addData("A", toggleB);
        telemetry.addData("servo",Claw.getPosition());



        //Left trigger is squeezed but right trigger is not so we are going down




        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Yaw (Z)", "%.2f Rad. (Heading)", orientation.getYaw(AngleUnit.DEGREES));





        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }



    @Override
    public void stop() {

    }
    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

}