package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
  

public class mecanum {
    // Drive Motors
    private static final WPI_VictorSPX frontL = new WPI_VictorSPX(2);
    private static final WPI_VictorSPX frontR = new WPI_VictorSPX(4);
    private static final WPI_VictorSPX backL = new WPI_VictorSPX(13);
    private static final WPI_VictorSPX backR = new WPI_VictorSPX(14);

    static Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    static Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    static Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    static Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    static MecanumDriveKinematics mecDrive = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    
    public static void setup(){
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    }
    public static void drive(double forwardJoystick, double sideJoystick){

        ChassisSpeeds chassisSpeed = new ChassisSpeeds(forwardJoystick,sideJoystick,0);
        MecanumDriveWheelSpeeds mecSpeeds = mecDrive.toWheelSpeeds(chassisSpeed);

        double frontLeft = mecSpeeds.frontLeftMetersPerSecond;
        double frontRight = mecSpeeds.frontRightMetersPerSecond;
        double backLeft = mecSpeeds.rearLeftMetersPerSecond;
        double backRight = mecSpeeds.rearRightMetersPerSecond;

        frontL.set(frontLeft);
        frontR.set(frontRight);
        backL.set(backLeft);
        backR.set(backRight);
    }
}
