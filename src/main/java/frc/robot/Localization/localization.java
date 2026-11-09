package frc.robot.Localization;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class localization {
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // Define following parameters before use
    private Pigeon2 m_gyro = CommandSwerveDrivetrain.pidgey; //GET REAL ID
    private SwerveModulePosition[] modulePositions = drivetrain.modulePositions;
    private boolean useMegaTag2 = true;
    

    private final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[] {
            new Translation2d(Inches.of(10), Inches.of(10)),
            new Translation2d(Inches.of(10), Inches.of(-10)),
            new Translation2d(Inches.of(-10), Inches.of(10)),
            new Translation2d(Inches.of(-10), Inches.of(-10))
            
    });

    private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            kBlueAlliancePerspectiveRotation,
            modulePositions,
            new Pose2d(0.4318, 4.021328, new Rotation2d(0)),
            new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] { 0.2, 0.2, 0.2 }),
            new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] { 0.6, 0.6, 0.5 }));

    /** Updates the robot pose using the given the camera's localization
     * @param cameraName
     * @return null
     */
    public void updateOdometry(String cameraName) {
        m_poseEstimator.update(m_gyro.getRotation2d(), modulePositions);

        boolean doRejectUpdate = false;

        if (useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        } else if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation(cameraName,
                    m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
            if (Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }
    }
    /**Returns the robot's estimated pose
     * @return Pose2d
     */
    public Pose2d getRobotPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
    /**
     * Returns the robot's estimated pose as a Field2d
     * @return Field2d
     */
    public Field2d getField2d() {
        Field2d field = new Field2d();
        field.setRobotPose(getRobotPose());
        return field;
    }
}
