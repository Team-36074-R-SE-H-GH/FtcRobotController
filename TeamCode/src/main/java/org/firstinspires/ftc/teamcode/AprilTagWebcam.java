
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedID) {
        if (detectedID == null) {
            return;
        }

        if (detectedID instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single =
                    (AprilTagSingleDetection) detectedID;

            if (single.metadata != null) {
                telemetry.addLine(String.format(
                        "\n==== (ID %d) %s",
                        single.id, single.metadata.name));

                telemetry.addLine(String.format(
                        "XYZ %6.1f %6.1f %6.1f (cm)",
                        detectedID.ftcPose.x,
                        detectedID.ftcPose.y,
                        detectedID.ftcPose.z));

                telemetry.addLine(String.format(
                        "PRY %6.1f %6.1f %6.1f (deg)",
                        detectedID.ftcPose.pitch,
                        detectedID.ftcPose.roll,
                        detectedID.ftcPose.yaw));

                telemetry.addLine(String.format(
                        "RBE %6.1f %6.1f %6.1f (cm, deg, deg)",
                        detectedID.ftcPose.range,
                        detectedID.ftcPose.bearing,
                        detectedID.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format(
                        "\n==== (ID %d) Unknown", single.id));

                telemetry.addLine(String.format(
                        "Center %6.0f %6.0f (pixels)",
                        single.center.x, single.center.y));
            }
        } else {
            telemetry.addLine("AprilTag cluster detected");

            telemetry.addLine(String.format(
                    "XYZ %6.1f %6.1f %6.1f (cm)",
                    detectedID.ftcPose.x,
                    detectedID.ftcPose.y,
                    detectedID.ftcPose.z));

            telemetry.addLine(String.format(
                    "RBE %6.1f %6.1f %6.1f (cm, deg, deg)",
                    detectedID.ftcPose.range,
                    detectedID.ftcPose.bearing,
                    detectedID.ftcPose.elevation));
        }
    }

    public double apriltagangle(AprilTagDetection detectedID) {
        if (detectedID == null || detectedID.ftcPose == null) {
            return 0;
        }

        return detectedID.ftcPose.bearing;
    }

    public static double apriltagheight(AprilTagDetection detectedID) {
        if (detectedID == null || detectedID.ftcPose == null) {
            return 0;
        }

        return detectedID.ftcPose.elevation;
    }
    public double Pythagorean_theorem(double x, double y){
        return sqrt(x*x + y*y);
    }
    public double angle_to_raised_hive_red(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if((cluster.metadata.name.charAt(0)== 'R')&&abs(cluster.ftcPose.roll) <= 90){
                    return cluster.ftcPose.bearing;
                }
            }
        }
        return -1;
    }
    public double angle_to_raised_hive_blue(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if((cluster.metadata.name.charAt(0)== 'B')&&abs(cluster.ftcPose.roll) <= 90){
                    return cluster.ftcPose.bearing;
                }
            }
        }
        return -1;
    }
    public double distance_to_raised_hive_red(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if((cluster.metadata.name.charAt(0)== 'R')&&abs(cluster.ftcPose.roll) <= 90){
                    return Pythagorean_theorem(cluster.ftcPose.x, cluster.ftcPose.z);
                }
            }
        }
        return -1;
    }
    public double distance_to_raised_hive_blue(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if((cluster.metadata.name.charAt(0)== 'B')&&abs(cluster.ftcPose.roll) <= 90){
                    return Pythagorean_theorem(cluster.ftcPose.x, cluster.ftcPose.z);
                }
            }
        }
        return -1;
    }
    public boolean is_red_scoring_down(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if(Objects.equals(cluster.metadata.name, "RED SCORING")&&abs(cluster.ftcPose.roll) <= 90){
                    return true;
                }
            }
        }
        return false;
    }
    public boolean is_red_audience_down(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if(Objects.equals(cluster.metadata.name, "RED AUDIENCE")&&abs(cluster.ftcPose.roll) <= 90){
                    return true;
                }
            }
        }
        return false;
    }
    public boolean is_blue_scoring_down(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if(Objects.equals(cluster.metadata.name, "BLUE SCORING")&&abs(cluster.ftcPose.roll) <= 90){
                    return true;
                }
            }
        }
        return false;
    }
    public boolean is_blue_audience_down(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if(Objects.equals(cluster.metadata.name, "BLUE AUDIENCE")&&abs(cluster.ftcPose.roll) <= 90){
                    return true;
                }
            }
        }
        return false;
    }


    public AprilTagDetection getTagBySpecificID(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single =
                        (AprilTagSingleDetection) detection;

                if (single.id == id) {
                    return single;
                }
            }
        }

        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
