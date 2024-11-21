package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="AutonomousMovement", group="FTC")
public class BasicMovements extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    @Override
    public void runOpMode() {

        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Set motor directions if needed
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Drive forward for 1 second
        moveForward(1000);

        // Turn right for 0.5 seconds
        turnRight(500);

        // Move backward for 1 second
        moveBackward(1000);

        // Turn left for 0.5 seconds
        turnLeft(500);

        // Stop the robot
        stopRobot();

    }



    // Method to move forward
    private void moveForward(int timeMs){

        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        telemetry.addData("Action", "Moving Forward");
        telemetry.update();

        sleep(timeMs);
        stopRobot();

    }



    // Method to move backward
    private void moveBackward(int timeMs) {

        leftDrive.setPower(-0.5);
        rightDrive.setPower(-0.5);
        telemetry.addData("Action", "Moving Backward");
        telemetry.update();

        sleep(timeMs);
        stopRobot();

    }



    // Method to turn right
    private void turnRight(int timeMs) {

        leftDrive.setPower(0.5);
        rightDrive.setPower(-0.5);
        telemetry.addData("Action", "Turning Right");
        telemetry.update();

        sleep(10000);
        stopRobot();

    }



    // Method to turn left
    private void turnLeft(int timeMs) {

        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        telemetry.addData("Action", "Turning Left");
        telemetry.update();

        sleep(timeMs);
        stopRobot();

    }



    // Method to stop the robot and display telemetry
    private void stopRobot() {

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        telemetry.addData("Action", "Stopped");
        telemetry.addData("Left Motor Position", leftDrive.getCurrentPosition());
        telemetry.addData("Right Motor Position", rightDrive.getCurrentPosition());
        telemetry.update();

        sleep(500); // Pause briefly to ensure the robot stops

    }

}