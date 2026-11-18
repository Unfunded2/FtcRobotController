package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * Road Runner localizer backed by a goBILDA Pinpoint v2.
 *
 * The existing pod coordinates are expressed in Road Runner coordinates.
 * Pinpoint's X pod is the parallel pod and its Y pod is the perpendicular pod.
 */
public class TwoWheelTrackingLocalizer implements Localizer {
    public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 2.05; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder speed)

    public static double PARALLEL_X = 0; // X is the forward and back direction
    public static double PARALLEL_Y = 7.5; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0.5;
    public static double PERPENDICULAR_Y = -7.5;

    private final GoBildaPinpointDriver pinpoint;
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVelocity = new Pose2d();

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(PARALLEL_Y, PERPENDICULAR_X, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(Pose2d poseEstimate) {
        this.poseEstimate = poseEstimate;
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                poseEstimate.getX(),
                poseEstimate.getY(),
                AngleUnit.RADIANS,
                poseEstimate.getHeading()));
    }

    @Override
    public void update() {
        pinpoint.update();

        Pose2D position = pinpoint.getPosition();
        poseEstimate = new Pose2d(
                position.getX(DistanceUnit.INCH),
                position.getY(DistanceUnit.INCH),
                position.getHeading(AngleUnit.RADIANS));
        poseVelocity = new Pose2d(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }

    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }
}
