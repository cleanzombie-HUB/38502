package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class PathsPedro {

    private final Pose start;
    private final Pose path1;
    private final Pose point2Start;
    private final Pose point2;
    private final Pose point3Start;
    private final Pose point3;
    private final Pose point4Start;
    private final Pose point4;
    private final Pose point5Start;
    private final Pose point5;
    private final Pose point6Start;
    private final Pose point6;
    private final Pose point7Start;
    private final Pose point7;
    private final Pose point8Start;
    private final Pose point8;
    private final Pose point9Start;
    private final Pose point9;
    private final Pose point10Start;
    private final Pose point10;

    public PathsPedro(subsystems.Alliance alliance) {
        this(alliance == subsystems.Alliance.BLUE);
    }

    public PathsPedro(boolean isBlue) {
        PoseFactory poseFactory = isBlue
                ? PoseFactory.degrees().mirrorX(70.75)
                : PoseFactory.degrees();

        start = poseFactory.of(56.6151, 8.6682, 90);
        path1 = poseFactory.of(20.0379, 44.9081, 180);
        point2Start = poseFactory.of(20.0379, 44.9081, 180);
        point2 = poseFactory.of(10.4318, 45.327, 180);
        point3Start = poseFactory.of(10.4318, 45.327, 180);
        point3 = poseFactory.of(22.1934, 50.4285, 180);
        point4Start = poseFactory.of(22.1934, 50.4285, 180);
        point4 = poseFactory.of(35.903, 103.0977, 270);
        point5Start = poseFactory.of(35.903, 103.0977, 270);
        point5 = poseFactory.of(58.5421, 129.8937, 270);
        point6Start = poseFactory.of(58.5421, 129.8937, 270);
        point6 = poseFactory.of(46.8319, 120.6858, 90);
        point7Start = poseFactory.of(46.8319, 120.6858, 90);
        point7 = poseFactory.of(46.8319, 128.9019, 90);
        point8Start = poseFactory.of(46.8319, 128.9019, 90);
        point8 = poseFactory.of(47.0145, 120.6858, 90);
        point9Start = poseFactory.of(47.0145, 120.6858, 90);
        point9 = poseFactory.of(57.9694, 129.4497, 270);
        point10Start = poseFactory.of(57.9694, 129.4497, 270);
        point10 = poseFactory.of(9.0381, 107.0372, 0);
    }

    public PathsPedro() {
        this(false);
    }

    public Pose getStartPose() {
        return start;
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }

    public Path path10() {
        return Paths.line(point10Start, point10).linear(point10Start, point10);
    }

    public static class Red extends PathsPedro {
        public Red() {
            super(false);
        }
    }

    public static class Blue extends PathsPedro {
        public Blue() {
            super(true);
        }
    }
}
