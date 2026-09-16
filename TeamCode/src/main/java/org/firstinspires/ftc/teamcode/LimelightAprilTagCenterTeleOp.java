package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "Limelight AprilTag Center", group = "TeamCode")
public class LimelightAprilTagCenterTeleOp extends LinearOpMode {
    private static final double DRIVE_SCALE = 0.65;
    private static final double TURN_SCALE = 0.55;
    private static final double CENTER_STRAFE_GAIN = 0.02;
    private static final double MAX_CENTER_STRAFE = 0.45;
    private static final double LOG_INTERVAL_SECONDS = 0.5;

    private DcMotor fLeftMotor;
    private DcMotor fRightMotor;
    private DcMotor bLeftMotor;
    private DcMotor bRightMotor;
    private Limelight3A limelight;

    private double lastTagLogTime = -1.0;

    @Override
    public void runOpMode() {
        fLeftMotor = hardwareMap.get(DcMotor.class, "fLeftMotor");
        fRightMotor = hardwareMap.get(DcMotor.class, "fRightMotor");
        bLeftMotor = hardwareMap.get(DcMotor.class, "bLeftMotor");
        bRightMotor = hardwareMap.get(DcMotor.class, "bRightMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        fLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        bLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        fRightMotor.setDirection(DcMotor.Direction.FORWARD);
        bRightMotor.setDirection(DcMotor.Direction.FORWARD);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Ready: left bumper to auto-center on AprilTag.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                double drive = -gamepad1.left_stick_y * DRIVE_SCALE;
                double strafe = -gamepad1.left_stick_x * DRIVE_SCALE;
                double turn = -gamepad1.right_stick_x * TURN_SCALE;

                boolean tagFound = false;
                int tagId = -1;
                double tagX = 0.0;
                double tagY = 0.0;

                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null && !fiducials.isEmpty()) {
                        LLResultTypes.FiducialResult tag = fiducials.get(0);
                        tagFound = true;
                        tagId = tag.getFiducialId();
                        tagX = tag.getTargetXDegrees();
                        tagY = tag.getTargetYDegrees();

                        if (lastTagLogTime < 0 || getRuntime() - lastTagLogTime >= LOG_INTERVAL_SECONDS) {
                            telemetry.log().add(String.format("Tag %d screen position -> x: %.2f deg, y: %.2f deg", tagId, tagX, tagY));
                            lastTagLogTime = getRuntime();
                        }
                    }
                }

                if (gamepad1.left_bumper && tagFound) {
                    strafe = Range.clip(-tagX * CENTER_STRAFE_GAIN, -MAX_CENTER_STRAFE, MAX_CENTER_STRAFE);
                    drive = 0.0;
                    turn = 0.0;
                    telemetry.addData("AutoCenter", "ON strafe=%.2f (x error %.2f deg)", strafe, tagX);
                } else {
                    telemetry.addData("AutoCenter", "OFF");
                }

                if (tagFound) {
                    telemetry.addData("Tag", "ID %d", tagId);
                    telemetry.addData("Tag Screen", "x: %.2f deg, y: %.2f deg", tagX, tagY);
                } else {
                    telemetry.addLine("Tag: not found");
                }

                moveRobot(drive, strafe, turn);
                telemetry.addData("Drive", "drive=%.2f strafe=%.2f turn=%.2f", drive, strafe, turn);
                telemetry.update();
            }
        } finally {
            limelight.stop();
            moveRobot(0.0, 0.0, 0.0);
        }
    }

    private void moveRobot(double x, double y, double yaw) {
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        fLeftMotor.setPower(frontLeftPower);
        fRightMotor.setPower(frontRightPower);
        bLeftMotor.setPower(backLeftPower);
        bRightMotor.setPower(backRightPower);
    }
}
