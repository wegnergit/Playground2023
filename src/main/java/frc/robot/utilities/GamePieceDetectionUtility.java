package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * 
 * A utility class for the game piece detection Lime Light camera
 * 
 */
public class GamePieceDetectionUtility {

    private String m_LimeLightName;
    private int m_pipeLine = 0;
    private boolean m_wasTriggerCreated = false;
    private boolean m_firstTime = true;

    /**
     * 
     * A utility class for the game piece detection Lime Light camera
     * 
     * @param LimeLightName The name of the camera1
     * @param pipeline 
     * 
     */
    public GamePieceDetectionUtility(String LimeLightName) {
        m_LimeLightName = LimeLightName;
    }

    public double get_tx() { //returns the amount of degrees off horizontally a game piece is from the center of the camera
        return LimelightHelpers.getTX(m_LimeLightName); 
    }

    public double get_ty() { //returns the amount of degrees off vertically a game piece is from the center of the camera
        return LimelightHelpers.getTY(m_LimeLightName);
    }

    public double get_ta() { //returns the percentage of the screen the game piece takes up
        return LimelightHelpers.getTA(m_LimeLightName);
    }

    public void switchPipeLine(int pipeline) {
        m_pipeLine = pipeline;
    
        // If never created trigger create it
        if(!m_wasTriggerCreated) {
            // Create a trigger to set pipeline once limelight is up and running and switch if not on correct pipeline
            createPipelineTrigger().onTrue(createSetPipelineIndexCommand());
            m_wasTriggerCreated = true;
        }
    }

    private Trigger createPipelineTrigger() {
        return new Trigger(
            () -> (isLatencyPipelinePresent() && 
                    // If turned off during match (TODO: handle if wish to switch modes)
                    (LimelightHelpers.getCurrentPipelineIndex(m_LimeLightName))!= m_pipeLine)
            );
    }

    private boolean isLatencyPipelinePresent() {
        boolean present = (LimelightHelpers.getLatency_Pipeline(m_LimeLightName) > 0.0);
        // Must return false first time so can get onTrue to trigger.
        if(m_firstTime) {
            m_firstTime = false;
            return false;
        } 
        // TODO Smartdashboard whether camera is connected (NOTE:  currently execute when trigger is create by switchpipeline())
        return present;
    }

    private Command createSetPipelineIndexCommand() {
        return new InstantCommand(() -> { setPipelineIndex(); })
                        // Need to run when disabled given will run at startup
                        .ignoringDisable(true);
    }

    private void setPipelineIndex() {
        // TODO somehow log so can keep track of
        DriverStation.reportWarning(m_LimeLightName+" set pipeline " +m_pipeLine, false);
        LimelightHelpers.setPipelineIndex(m_LimeLightName, m_pipeLine);
    }


}





