package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotMap;
import frc.robot.util.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;
    
    private final SwerveModule[] swerveModules;
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    
    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;
    
    private final Pigeon2 gyro;
    private double heading;

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }
    
    public Drivetrain() {
        frontLeft = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
                    RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID, 0);
        frontRight = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
                    RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID, 0);
        backLeft = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
                    RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID, 0);
        backRight = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
                    RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID, 0);

        swerveModules = new SwerveModule[] {
            frontLeft, frontRight, backLeft, backRight
        };
        positions = new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()
        };
        // set states to the values calculated for 0 movement
        states = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        
        gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_NAME);
        gyro.setYaw(0);
    }
    
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    
    public Rotation2d getHeadingAsRotation2d() {
        return gyro.getRotation2d();
    }
    
    public void updateModulePositions() {
        for (int i = 0; i < 4; i++)
            positions[i] = swerveModules[i].getPosition();
    }
    
    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < 4; i++)
            swerveModules[i].setDesiredState(desiredStates[i]);
    }
    
    // in radians/s
    public void drive(Translation2d translation, double rotation,
            boolean fieldOriented, Translation2d centerOfRotation) {

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        
        // not field oriented then joystick direction is robot direction
        ChassisSpeeds robotRelativeSpeeds = fieldRelativeSpeeds;
        if (fieldOriented)
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d());
        
        SmartDashboard.putNumber("rotation thingy", rotation);
        SmartDashboard.putNumber("rotation speed", robotRelativeSpeeds.omegaRadiansPerSecond);
        
        states = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SmartDashboard.putNumber("module 0 rotation deg", states[0].angle.getDegrees());
        SmartDashboard.putNumber("module 1 rotation deg", states[1].angle.getDegrees());
        SmartDashboard.putNumber("module 2 rotation deg", states[2].angle.getDegrees());
        SmartDashboard.putNumber("module 3 rotation deg", states[3].angle.getDegrees());

        for (int i = 0; i < 4; i++)
            states[i] = SwerveModuleState.optimize(states[i], new Rotation2d(swerveModules[i].getCANCoderRadians()));
        
        setSwerveModuleStates(states);
    }
}
