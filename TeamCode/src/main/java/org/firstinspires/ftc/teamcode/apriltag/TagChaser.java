package org.firstinspires.ftc.teamcode.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;
import org.openftc.apriltag.AprilTagDetection;

public class TagChaser {

    private PFinder pFinder;
    private boolean isToggled = false;
    private AprilTagDetection latestDetection;
    private PIDController rotController = new PIDController(0.1, 0, 0);
    private PIDController yController = new PIDController(0.1, 0, 0);
    private PIDController xController = new PIDController(0.1, 0, 0);

    private double rotPower = 0;
    private double xPower = 0;
    private double yPower = 0;
    private double specifiedID = 0;



    public TagChaser(AprilTagDetection latestDetection, PFinder pFinder) {

        this.pFinder = pFinder;
        this.latestDetection = latestDetection;
    }

    public void loop() {

        Orientation rot = Orientation.getOrientation(latestDetection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        if (isToggled && latestDetection != null && latestDetection.id == specifiedID){
            rotPower = rotController.calculate(rot.firstAngle);
            xPower = yController.calculate(latestDetection.pose.x);
            yPower = xController.calculate(5 - latestDetection.pose.y);

            pFinder.drive(new Pose( -xPower * 0.5, -yPower * 0.5, -rotPower*0.5));

            if(Math.abs(rotPower) < 0.1 && Math.abs(xPower) < 0.1 && Math.abs(yPower) < 0.){
                isToggled = false;
            }

        }

    }

    public void toggle() {
        isToggled = !isToggled;
    }

    public void setID(double id){
        specifiedID = id;
    }
}
