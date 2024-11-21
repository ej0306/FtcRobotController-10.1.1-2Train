package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutonomousMovementFourWheels", group="FTC")
public class BasicMovementsUpdated extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;


    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize Hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        // Set motor directions if needed
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (Driver presses PLAY)
        waitForStart();

        // Drive forward for 1 second
        moveForward(1000);

        // Turn right for 0.5 seconds
        turnRight(500);

        // Move Backward for 1 second
        moveBackward(1000);

        // Turn left for 0.5 second
        turnLeft(500);

        // Stop the robot
        stopRobot();
    }


    // Method to move forward
    private void  moveForward(int timeMs){
        setMotorPower(0.5);
        telemetry.addData("Action", "Moving Forward");
        telemetry.update();
        sleep(timeMs);
        stopRobot();
    }

    // Method to move backward
    private void  moveBackward(int timeMs) {
        setMotorPower(-0.5);
        telemetry.addData("Action", "Moving Backward");
        telemetry.update();
        sleep(timeMs);
        stopRobot();
    }

    // Method to turn right
    private void turnRight(int timeMs){
        leftFrontDrive.setPower(0.5);
        leftRearDrive.setPower(0.5);
        rightFrontDrive.setPower(-0.5);
        rightRearDrive.setPower(-0.5);
        telemetry.addData("Action", "Turning Right");
        telemetry.update();
        sleep(timeMs);
        stopRobot();
    }

    // Method to turn left
    private void turnLeft(int timeMs){
        leftFrontDrive.setPower(-0.5);
        leftRearDrive.setPower(-0.5);
        rightFrontDrive.setPower(0.5);
        rightRearDrive.setPower(0.5);
        telemetry.addData("Action", "Turning Left");
        telemetry.update();
        sleep(timeMs);
        stopRobot();
    }

    // Helper method to set power for all motors
    private void setMotorPower(double power){
        leftFrontDrive.setPower(power);
        leftRearDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightRearDrive.setPower(power);
    }

    // Method to stop the robot
    private void stopRobot(){
        setMotorPower(0);
        telemetry.addData("Action", "Stopped");
        telemetry.update();
        sleep(500); //Pause briefly to ensure the robot stops
    }

}




