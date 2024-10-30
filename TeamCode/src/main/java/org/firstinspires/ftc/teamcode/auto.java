
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Autonomous(name = "auto", group = "Linear Opmode")
    public class auto extends LinearOpMode {
        private DcMotorEx leftFront, rightFront, leftRear, rightRear;
        private ElapsedTime runtime = new ElapsedTime();

        // Constants
        private static final double INCHES_TO_TICKS = 1120 / (4 * Math.PI); // Example for a 4-inch wheel
        private static final double ROTATE_TICKS = 1120 / 360; // Adjust based on your robot

        @Override
        public void runOpMode() throws InterruptedException {
            // Initialize the motors
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

            // Set motor direction if needed
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            rightRear.setDirection(DcMotor.Direction.REVERSE);

            waitForStart();

            // Move forward 23 inches
            moveForward(23);

            // Strafe left 20 inches
            strafeLeft(20);

            // Move forward 33 inches
            moveForward(33);

            // Rotate 90 degrees
            rotate(90);

            // Move forward 5 inches
            moveForward(5);
        }

        private void moveForward(double inches) {
            int ticks = (int) (inches * INCHES_TO_TICKS);
            setMotorTargetPosition(ticks, ticks, ticks, ticks);
            setMotorPower(0.5);
            waitForMotors();
            stopMotors();
        }

        private void strafeLeft(double inches) {
            int ticks = (int) (inches * INCHES_TO_TICKS);
            setMotorTargetPosition(-ticks, ticks, ticks, -ticks);
            setMotorPower(0.5);
            waitForMotors();
            stopMotors();
        }

        private void rotate(double degrees) {
            int ticks = (int) (degrees * ROTATE_TICKS);
            setMotorTargetPosition(ticks, -ticks, ticks, -ticks);
            setMotorPower(0.5);
            waitForMotors();
            stopMotors();
        }

        private void setMotorTargetPosition(int frontLeftTicks, int frontRightTicks, int rearLeftTicks, int rearRightTicks) {
            leftFront.setTargetPosition(leftFront.getCurrentPosition() + frontLeftTicks);
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + frontRightTicks);
            leftRear.setTargetPosition(leftRear.getCurrentPosition() + rearLeftTicks);
            rightRear.setTargetPosition(rightRear.getCurrentPosition() + rearRightTicks);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private void setMotorPower(double power) {
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);
        }

        private void waitForMotors() {
            while (opModeIsActive() && (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())) {
                // Wait for all motors to finish
            }
        }

        private void stopMotors() {
            setMotorPower(0);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
