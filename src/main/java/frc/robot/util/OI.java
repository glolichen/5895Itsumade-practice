package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;

public class OI {
    private static OI instance;
    private PS4Controller controller;

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
    
    public OI() {
        controller = new PS4Controller(0);
    }
    
    public double getForward() {
        // return -controller.getLeftY();
        return -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
    }
    
    public double getStrafe() {
        return -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
    }
    
    public Translation2d getSwerveTranslation() {
        return new Translation2d(getForward(), getStrafe());
    }
    
    public double getRotation() {
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        return (leftRotation - rightRotation) / 2.0; // TODO: convert to rad/s
    }
}
