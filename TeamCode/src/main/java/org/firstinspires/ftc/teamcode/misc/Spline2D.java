package org.firstinspires.ftc.teamcode.misc;

import java.util.ArrayList;
import java.util.List;

class Position2D {
    double x;
    double y;
}

class SplineSection {
    Position2D startPoint;
    Position2D endPoint;
    Position2D lerpPoint1;
    Position2D lerpPoint2;
}

class BezierCurveSpline {
    List<SplineSection> splineSections = new ArrayList<SplineSection>();
    String name;

    BezierCurveSpline(String splineName) {
        name = splineName;
    }

}

public class Spline2D {
    
    public List<BezierCurveSpline> Splines = new ArrayList<BezierCurveSpline>();

    void AddSpline(String splineName) {
        Splines.add(new BezierCurveSpline(splineName));
    }

    /**
     * use a cad software to visualize the splines
     */
    void AddSplineSection(double startX, double startY, double endX, double endY, double lerp1X, double lerp1Y, double lerp2X, double lerp2Y) {
        BezierCurveSpline activeSpline = Splines.get(Splines.size() - 1);
        SplineSection newSection = new SplineSection();

        newSection.startPoint.x = startX;
        newSection.startPoint.y = startY;

        newSection.endPoint.x = endX;
        newSection.endPoint.y = endY;

        newSection.lerpPoint1.x = lerp1X;
        newSection.lerpPoint1.y = lerp1Y;

        newSection.lerpPoint2.x = lerp2X;
        newSection.lerpPoint2.y = lerp2Y;

        activeSpline.splineSections.add(newSection);
    }

    public void followSpline(String name) {

        int splineIndex = -1;
        for (int i = 0; i < Splines.size(); i++) {
            if (Splines.get(i).name.equals(name)) {
                splineIndex = i;
            }
        }
        if (splineIndex == -1)
            throw new ArithmeticException("name entered does not match any splines in list. aka get good skill issue detected no cap");


    }
    
    
}
