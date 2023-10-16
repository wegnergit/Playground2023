package frc.robot.simulation;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.arm.ArmSubsystem;

public class MechanismSimulator {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d m_arm;

    private final ArmSubsystem arm;

    private final SwerveDrive swerve;

    private static double armYOffset = 0;
    private static double armZOffset = 0;
    private static double armXOffset = 0;

    /**
     * Simulates the arm and elevator systems in simulation, in a 2d window.
     * 
     * @param arm Subsystem for the arm.
     * @param elevator Subsystem for the elevator.
     */
    public MechanismSimulator(ArmSubsystem arm, /*  PitchIntakeSubsystem intake,*/ SwerveDrive swerve){
        this.arm = arm;
        //this.intake = intake;
        this.swerve = swerve;

        mech = new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100));
        root = mech.getRoot("arm", Units.inchesToMeters(7.35), Units.inchesToMeters(10));
        mech.getRoot("Robot", 0, Units.inchesToMeters(7)).append(
            new MechanismLigament2d("frame", Units.inchesToMeters(26+7.5), 0, 5, new Color8Bit(Color.kBlanchedAlmond))
        );
        
        // Adds arm to the elevator in simulation
        m_arm =
                root.append(
                new MechanismLigament2d(
                    "Arm",
                    Units.inchesToMeters(24.719),
                    arm.getPosition(),
                    6,
                    new Color8Bit(Color.kYellow)
                    )
                );
        
        //Adds manipulator to the arm in simulation
        
        // Adds intake to the root robot simulation
        // m_intake =
        //     root.append(
        //         new MechanismLigament2d(
        //             "Intake",
        //             Units.inchesToMeters(9.4),
        //             this.intake.getEncoderPosition(),
        //             6,
        //             new Color8Bit(Color.kBlueViolet)
        //         )
        //     );
        
    }
    
    /**
     * <h3>periodic</h3>
     * 
     * Constantly updates current simulation angle to the display and update the positions for Advantage Scope
     */
    public void periodic(){
        m_arm.setAngle(arm.getPosition());
        //m_intake.setAngle(intake.getEncoderPosition());

        
        // Arm Position in Advantage Scope
        double[] armPosition = {
            armXOffset + swerve.getPose().getX(),
            armYOffset + swerve.getPose().getY(),
            1,
            0,
            arm.getPosition(),
            swerve.getHeadingDegrees()
        };
        // Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/ArmPos", armPosition);


        // Sends system simulations to logger
        // Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/Mech2d", mech);
    }
}
