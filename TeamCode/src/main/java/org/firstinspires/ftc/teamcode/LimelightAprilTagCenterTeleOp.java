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
    private static final double CENTER_STRAFE_GAIN = 0.02;
    private static final double MAX_CENTER_STRAFE = 0.45;
    private static final double CENTERED_MARGIN_DEGREES = 1.5;
    private static final double HIVE_TILT_MARGIN_DEGREES = 1.0;
    private static final double LOG_INTERVAL_SECONDS = 0.5;

    private DcMotor fLeftMotor;
    private DcMotor fRightMotor;
    private DcMotor bLeftMotor;
    private DcMotor bRightMotor;
    private Limelight3A limelight;

    private double lastTagLogTime = -1.0;
    private boolean wasAButtonPressed = false;
    private boolean wasRightBumperPressed = false;
    private boolean autoCenterActive = false;
    private Alliance selectedAlliance = Alliance.BLUE;
    private boolean lastAudienceClusterComplete = false;
    private boolean lastScoringClusterComplete = false;
    private HiveTiltState lastHiveTiltState = HiveTiltState.UNKNOWN;

    private enum Alliance {
        RED,
        BLUE
    }

    private enum HiveTiltState {
        UNKNOWN,
        AUDIENCE_UP_SCORING_DOWN,
        SCORING_UP_AUDIENCE_DOWN
    }

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

        telemetry.addLine("Ready: A toggles alliance, RB starts auto-center, LB slow mode.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                boolean aPressed = gamepad1.a;
                if (aPressed && !wasAButtonPressed) {
                    selectedAlliance = (selectedAlliance == Alliance.BLUE) ? Alliance.RED : Alliance.BLUE;
                    telemetry.log().add("Alliance selected: " + selectedAlliance);
                    lastAudienceClusterComplete = false;
                    lastScoringClusterComplete = false;
                    lastHiveTiltState = HiveTiltState.UNKNOWN;
                }
                wasAButtonPressed = aPressed;

                boolean rightBumperPressed = gamepad1.right_bumper;
                if (rightBumperPressed && !wasRightBumperPressed) {
                    autoCenterActive = true;
                    telemetry.log().add("Auto-center started for " + selectedAlliance + " alliance");
                }
                wasRightBumperPressed = rightBumperPressed;

                double speedMul = gamepad1.left_bumper ? 0.5 : 1.0;
                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double strafe = gamepad1.right_trigger - gamepad1.left_trigger;

                boolean tagFound = false;
                int tagId = -1;
                double tagX = 0.0;
                double tagY = 0.0;
                int audienceTagCount = 0;
                int scoringTagCount = 0;
                double audienceTagYSum = 0.0;
                double scoringTagYSum = 0.0;

                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null) {
                        for (LLResultTypes.FiducialResult tag : fiducials) {
                            int currentTagId = tag.getFiducialId();
                            if (isTagInSelectedAlliance(currentTagId, selectedAlliance)) {
                                double currentTagX = tag.getTargetXDegrees();
                                double currentTagY = tag.getTargetYDegrees();
                                if (!tagFound || Math.abs(currentTagX) < Math.abs(tagX)) {
                                    tagFound = true;
                                    tagId = currentTagId;
                                    tagX = currentTagX;
                                    tagY = currentTagY;
                                }
                                if (isAudienceTagForAlliance(currentTagId, selectedAlliance)) {
                                    audienceTagCount++;
                                    audienceTagYSum += currentTagY;
                                } else if (isScoringTagForAlliance(currentTagId, selectedAlliance)) {
                                    scoringTagCount++;
                                    scoringTagYSum += currentTagY;
                                }
                            }
                        }

                        if (tagFound && (lastTagLogTime < 0 || getRuntime() - lastTagLogTime >= LOG_INTERVAL_SECONDS)) {
                            telemetry.log().add(String.format("%s tag %d screen position -> x: %.2f deg, y: %.2f deg",
                                    selectedAlliance, tagId, tagX, tagY));
                            lastTagLogTime = getRuntime();
                        }
                    }
                }

                boolean audienceClusterComplete = (audienceTagCount == 4);
                boolean scoringClusterComplete = (scoringTagCount == 4);
                if (audienceClusterComplete != lastAudienceClusterComplete) {
                    telemetry.log().add(getAudienceClusterLabel(selectedAlliance) + (audienceClusterComplete ? " cluster: complete (4/4)" : " cluster: incomplete"));
                    lastAudienceClusterComplete = audienceClusterComplete;
                }
                if (scoringClusterComplete != lastScoringClusterComplete) {
                    telemetry.log().add(getScoringClusterLabel(selectedAlliance) + (scoringClusterComplete ? " cluster: complete (4/4)" : " cluster: incomplete"));
                    lastScoringClusterComplete = scoringClusterComplete;
                }

                HiveTiltState currentHiveTiltState = HiveTiltState.UNKNOWN;
                if (audienceTagCount > 0 && scoringTagCount > 0) {
                    double audienceAvgY = audienceTagYSum / audienceTagCount;
                    double scoringAvgY = scoringTagYSum / scoringTagCount;
                    double yDiff = audienceAvgY - scoringAvgY;
                    if (Math.abs(yDiff) > HIVE_TILT_MARGIN_DEGREES) {
                        currentHiveTiltState = yDiff > 0
                                ? HiveTiltState.AUDIENCE_UP_SCORING_DOWN
                                : HiveTiltState.SCORING_UP_AUDIENCE_DOWN;
                    }
                }
                if (currentHiveTiltState != lastHiveTiltState) {
                    telemetry.log().add("Hive state (" + selectedAlliance + "): " + getHiveTiltStateLabel(currentHiveTiltState));
                    if (lastHiveTiltState != HiveTiltState.UNKNOWN && currentHiveTiltState != HiveTiltState.UNKNOWN) {
                        telemetry.log().add("Hive moved on " + selectedAlliance + " alliance side.");
                    }
                    lastHiveTiltState = currentHiveTiltState;
                }

                if (autoCenterActive) {
                    if (tagFound) {
                        if (Math.abs(tagX) <= CENTERED_MARGIN_DEGREES) {
                            autoCenterActive = false;
                            telemetry.log().add(String.format("Auto-center complete for tag %d (x error %.2f deg)", tagId, tagX));
                            telemetry.addData("AutoCenter", "DONE (x error %.2f deg)", tagX);
                        } else {
                            strafe = Range.clip(-tagX * CENTER_STRAFE_GAIN, -MAX_CENTER_STRAFE, MAX_CENTER_STRAFE);
                            forward = 0.0;
                            turn = 0.0;
                            telemetry.addData("AutoCenter", "ON strafe=%.2f (x error %.2f deg)", strafe, tagX);
                        }
                    } else {
                        telemetry.addData("AutoCenter", "ON waiting for %s tag", selectedAlliance);
                    }
                } else {
                    telemetry.addData("AutoCenter", "OFF");
                }

                telemetry.addData("Alliance", selectedAlliance);
                telemetry.addData(getAudienceClusterLabel(selectedAlliance), "%d/4 tags (%s)", audienceTagCount, audienceClusterComplete ? "COMPLETE" : "PARTIAL");
                telemetry.addData(getScoringClusterLabel(selectedAlliance), "%d/4 tags (%s)", scoringTagCount, scoringClusterComplete ? "COMPLETE" : "PARTIAL");
                telemetry.addData("Hive Tilt", getHiveTiltStateLabel(currentHiveTiltState));
                if (tagFound) {
                    telemetry.addData("Tag", "ID %d", tagId);
                    telemetry.addData("Tag Screen", "x: %.2f deg, y: %.2f deg", tagX, tagY);
                } else {
                    telemetry.addLine("Tag: not found");
                }

                bLeftMotor.setPower((forward + turn - strafe) * speedMul);
                fLeftMotor.setPower((forward + turn + strafe) * speedMul);
                bRightMotor.setPower((forward - turn + strafe) * speedMul);
                fRightMotor.setPower((forward - turn - strafe) * speedMul);
                telemetry.addData("Drive", "fwd=%.2f strafe=%.2f turn=%.2f mul=%.2f", forward, strafe, turn, speedMul);
                telemetry.update();
            }
        } finally {
            limelight.stop();
            bLeftMotor.setPower(0.0);
            fLeftMotor.setPower(0.0);
            bRightMotor.setPower(0.0);
            fRightMotor.setPower(0.0);
        }
    }

    private boolean isTagInSelectedAlliance(int tagId, Alliance alliance) {
        return isAudienceTagForAlliance(tagId, alliance) || isScoringTagForAlliance(tagId, alliance);
    }

    private boolean isAudienceTagForAlliance(int tagId, Alliance alliance) {
        return alliance == Alliance.BLUE ? (tagId >= 38 && tagId <= 41) : (tagId >= 34 && tagId <= 37);
    }

    private boolean isScoringTagForAlliance(int tagId, Alliance alliance) {
        return alliance == Alliance.BLUE ? (tagId >= 42 && tagId <= 45) : (tagId >= 30 && tagId <= 33);
    }

    private String getAudienceClusterLabel(Alliance alliance) {
        return alliance == Alliance.BLUE ? "Blue Audience" : "Red Audience";
    }

    private String getScoringClusterLabel(Alliance alliance) {
        return alliance == Alliance.BLUE ? "Blue Scoring" : "Red Scoring";
    }

    private String getHiveTiltStateLabel(HiveTiltState state) {
        switch (state) {
            case AUDIENCE_UP_SCORING_DOWN:
                return "Audience UP / Scoring DOWN";
            case SCORING_UP_AUDIENCE_DOWN:
                return "Scoring UP / Audience DOWN";
            default:
                return "Unknown";
        }
    }
}
