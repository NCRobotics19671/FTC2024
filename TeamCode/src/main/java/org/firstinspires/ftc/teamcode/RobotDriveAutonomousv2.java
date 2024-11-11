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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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


@Autonomous
public class RobotDriveAutonomousv2 extends OpMode {
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor Arm = null;
    private DcMotor Alien = null;


    double kP = 0.4; // to be tuned
    double kI = 0.15;  // to be tuned]
    double kD = 0.00;  // to be tuned

    // PID variables
    double integral = 0;
    double previousError = 0;
    double deriv = 1;
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

    boolean toggleA = false;

    double circumference = Math.PI * 2.95276;

    double MOTOR_TICK_COUNTS = 336;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 3; // Up
        usbFacingDirectionPosition = 0; // Forward

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;


        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        Arm = hardwareMap.dcMotor.get("Arm");
        Alien = hardwareMap.dcMotor.get("Alien");


        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Alien.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

       driveYdir(15,0.3);
       //drop off sample
       driveYdir(7,-0.3);
       turnToAngle(-90);
       driveXdir(12,0.3);
       driveYdir(10,-0.3);







    }

    @Override
    public void loop() {


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Yaw (Z)", "%.2f Rad. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Alien", Alien.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
        telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
        telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
        telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());

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

    public void turnToAngle(double targetAngle) {
        double turnco = 0;
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double botheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (botheading > targetAngle) {
            turnco = -1;
        }
        else if (botheading < targetAngle){turnco = 1;}

        motorBackLeft.setPower(-turnco * 0.5);
        motorBackRight.setPower(turnco * 0.5);
        motorFrontLeft.setPower(-turnco * 0.5);
        motorFrontRight.setPower(turnco * 0.5);
        if (turnco == -1){
        while (botheading > targetAngle)
        {botheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}}
        else if (turnco == 1){ while (botheading < targetAngle)
        {botheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}}
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        /*double t = getRuntime();
        double error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double derivative;
        while (Math.abs(error) > 0.5) { // 1 degree tolerance
            boolean ready = false;
            error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            integral += error * 0.01; // Assume loop time of 0.01 seconds
            derivative = (error - previousError) / 0.01;

            double power = Math.min((kP * error + kI * integral + kD * derivative),1);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(power);

            previousError = error;

            telemetry.addData("Error", error);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Power", power);
            telemetry.update();

            while (!ready) {
                if (getRuntime() - t >= 0.01) {
                    ready = true;
                    telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                }
            }
            // Stop the motors
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
*/

    }


    // Reverse the right side motors
    // Reverse left motors if you are using NeveRests


    public void driveYdir(double distance, double power) {
        //

        distance = distance/0.75;
        double rpm = power * 500;
        double speed = ((rpm*3*Math.PI)/60);
        double inverse = 1/speed;
        double addtime = distance * inverse;
        //double StrafeRotations = 30/circumference;
        //int StrafeDrivingTarget =  (int)(StrafeRotations*MOTOR_TICK_COUNTS);
        double startt = getRuntime();
        double t = getRuntime();
  

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      


        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
        
        double endt = (getRuntime() + addtime);
        while (t < endt) {
            telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
            telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
            telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());
            t = getRuntime();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void driveXdir(double distance, double power) {

        //distance = distance/0.75;
        double rpm = power * 500;
        double speed = ((rpm*3*Math.PI)/60);
        double inverse = 1/speed;
        double addtime = distance * inverse;
        //double StrafeRotations = 30/circumference;
        //int StrafeDrivingTarget =  (int)(StrafeRotations*MOTOR_TICK_COUNTS);
        double startt = getRuntime();
        double t = getRuntime();


        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(-power);

        double endt = (getRuntime() + addtime);
        while (t < endt) {
            telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
            telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
            telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());
            t = getRuntime();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public void driveArm(int angle, double power) {


        //double StrafeRotations = 30/circumference;
        //int StrafeDrivingTarget =  (int)(StrafeRotations*MOTOR_TICK_COUNTS);
        double rotationsNeeded = angle/360.0;
        int encoderDrivingTarget = (int)(rotationsNeeded*10000);
        int target = encoderDrivingTarget;




        Arm.setTargetPosition(target);

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm.setPower(power);


        while (Arm.isBusy()) {

        }
        Arm.setPower(0);


    }

    public void driveAlien(float distance, double power) {


        //double StrafeRotations = 30/circumference;
        //int StrafeDrivingTarget =  (int)(StrafeRotations*MOTOR_TICK_COUNTS);
        double rotationsNeeded = distance/360.0;
        int encoderDrivingTarget = (int)(distance*625);
        int target = encoderDrivingTarget;




        Alien.setTargetPosition(target);

        Alien.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Alien.setPower(power);


        while (Alien.isBusy()) {

        }
        Alien.setPower(0);


    }
}
