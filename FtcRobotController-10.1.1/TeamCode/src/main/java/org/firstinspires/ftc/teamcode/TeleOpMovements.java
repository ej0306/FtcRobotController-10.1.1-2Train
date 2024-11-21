package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOpMovements", group="FTC")
public class TeleOpMovements extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    @Override
    public void init() {
        // Initialize drivetrain motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Gamepad inputs
        double drive = -gamepad1.left_stick_y; // Forward/backward motion
        double strafe = gamepad1.left_stick_x; // Left/right strafing
        double turn = gamepad1.right_stick_x; // Rotation (turning)

        // Apply dead zones to avoid accidental drift
        if (Math.abs(drive) < 0.1) drive = 0;
        if (Math.abs(strafe) < 0.1) strafe = 0;
        if (Math.abs(turn) < 0.1) turn = 0;

        // Calculate motor powers for mecanum drive
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double leftFrontPower = (drive + strafe + turn) / denominator;
        double leftRearPower = (drive - strafe + turn) / denominator;
        double rightFrontPower = (drive - strafe - turn) / denominator;
        double rightRearPower = (drive + strafe - turn) / denominator;

        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightRearDrive.setPower(rightRearPower);

        // Telemetry for debugging
        telemetry.addData("Drive (Forward/Backward)", drive);
        telemetry.addData("Strafe (Left/Right)", strafe);
        telemetry.addData("Turn (Rotation)", turn);
        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Left Rear Power", leftRearPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Right Rear Power", rightRearPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all drivetrain motors
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
