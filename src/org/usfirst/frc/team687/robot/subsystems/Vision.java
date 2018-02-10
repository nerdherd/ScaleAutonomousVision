package org.usfirst.frc.team687.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Vision extends Subsystem {

    /** _actual means field dimensions
     *  _pixel means camera pixel dimensions
     */

	private AHRS navx;
	private double target_length_actual = 36; //inches
	
	
	private double camera_FOV_pixel = 360;
	private double camera_FOV_degree = 65; 
	
	//JeVois Values
	private double target_left_pixel = 20;
	private double target_right_pixel = 120;
	
//	private double 
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	navx = new AHRS(SerialPort.Port.kMXP);
    }
    
    public double[] getTargetCenter() {
    	double target_length_pixel = Math.abs(target_left_pixel - target_right_pixel);
    	double ratio_actual_to_pixel = target_length_pixel/target_length_actual;
    	double X_length_pixel = Math.abs(target_left_pixel - (camera_FOV_pixel/2));
    	double X_length_actual = ratio_actual_to_pixel * X_length_pixel; //x
    	
    	double ratio_degree_to_actual = camera_FOV_degree/camera_FOV_pixel;
    	double beta = ratio_degree_to_actual * target_length_pixel; //b
    	
    	double Y_length_actual = X_length_actual/Math.tan(beta); //y
    	
    	double[] target_coordinate = new double[2];
    	target_coordinate[0] = -X_length_actual + (1/2) * target_length_pixel;
    	target_coordinate[1] = Y_length_actual;
    	
    	return target_coordinate;
    }
    
    public double getTargetAngle() {
    	double alpha = navx.getYaw(); //a
    	
    	double target_length_pixel = Math.abs(target_left_pixel - target_right_pixel);
    	double ratio_degree_to_actual = camera_FOV_degree/camera_FOV_pixel;
    	double beta = ratio_degree_to_actual * target_length_pixel; //b
    	
    	return beta+alpha;
    }
}

