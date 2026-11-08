package org.firstinspires.ftc.teamcode.PedroPathing.pedro.procedures;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FlowersAuto", group = "Autonomous")
    public class FlowersAuto {

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(61.4774, 131.8258, 90);
        private final Pose path1 = poseFactory.of(61.4774, 131.3984, 270);
        private final Pose point2 = poseFactory.of(60.6484, 107.9339, 270);
        private final Pose point3 = poseFactory.of(37.721, 110.9548, 126.6142);
        private final Pose point3Control1 = poseFactory.of(75.2548, 140.3387, 0);
        private final Pose point3Control2 = poseFactory.of(14.8387, 140.9419, 0);
        private final Pose point4 = poseFactory.of(60.3726, 108.3629, 270);
        private final Pose point5 = poseFactory.of(10.0339, 30.8984, 100.4891);
        private final Pose point5Control1 = poseFactory.of(3.4419, 64.7387, 0);
        private final Pose point6 = poseFactory.of(57.9758, 31.8806, 90);
        private final Pose point7 = poseFactory.of(17.3129, 119.4742, -65.0982);

        public Path path1() {
            return Paths.line(start, path1).constant(path1);
        }

        public Path path2() {
            return Paths.line(path1, point2).constant(point2);
        }

        public Path path3() {
            return Paths.curve(point2, point3Control1, point3Control2, point3).reverseTangent();
        }

        public Path path4() {
            return Paths.line(point3, point4).constant(point4);
        }

        public Path path5() {
            return Paths.curve(point4, point5Control1, point5).reverseTangent();
        }

        public Path path6() {
            return Paths.line(point5, point6).constant(point6);
        }

        public Path path7() {
            return Paths.line(point6, point7).reverseTangent();
        }
    }


