package com.kircherelectronics.androidtiltcompensation;

import java.util.Arrays;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.app.Activity;
import android.util.Log;
import android.view.Menu;

/**
 * Compensate for the tilt of the accelerometer by calculating the gravity
 * component of the accelerometer signal from the Cardan angles.
 * 
 * Any vector sensor measuring a known, constant, homogonous field may be used
 * to solved for an orientation relative to that field. However, for any given
 * measurement there will not be a unique sensor orientation solution, instead
 * there will be infinite solutions represented by all those orientations
 * achieved by the rotation the true orientation around an axis parallel with
 * the field. An accelerometer alone will tell you pitch and roll; not yaw
 * because gravity is parallel to yaw (z) and a magnetometer alone will tell you
 * roll and yaw; not pitch because the Earth’s magnetic field is parallel to
 * pitch (x)*.
 * 
 * This leads to the disappointing consequence that the pitch can never be
 * determined accurately while the device is experiencing linear acceleration
 * with a magnetometer/accelerometer sensor fusion.
 * 
 * @author Kaleb
 * 
 */
public class TiltCompensationActivity extends Activity implements
		SensorEventListener
{
	private float timestampAccel;
	private float timestampAccelOld;

	private float timestampMag;
	private float timestampMagOld;

	// Raw accelerometer data
	private float[] inputAccel = new float[3];
	private float[] inputMag = new float[3];

	private String tag = "Sensor Rotation";

	private SensorManager sensorManager;

	@Override
	protected void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_tilt_compensation);

		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu)
	{
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_tilt_compensation, menu);
		return true;
	}

	@Override
	protected void onResume()
	{
		super.onResume();

		// Register for sensor updates.
		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_FASTEST);

		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
				SensorManager.SENSOR_DELAY_FASTEST);
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy)
	{
		// TODO Auto-generated method stub

	}

	@Override
	public void onSensorChanged(SensorEvent event)
	{
		if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
		{
			timestampAccel = event.timestamp;

			// Make sure the timestamp for the event has changed.
			if (timestampAccel != timestampAccelOld)
			{
				// Get a local copy of the sensor values
				System.arraycopy(event.values, 0, inputAccel, 0,
						event.values.length);
			}

		}

		if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
		{
			timestampMag = event.timestamp;

			// Make sure the timestamp for the event has changed.
			if (timestampMag != timestampMagOld)
			{
				// Get a local copy of the sensor values
				System.arraycopy(event.values, 0, inputMag, 0,
						event.values.length);

				// The rotation matrix R transforming a vector from the device
				// coordinate system to the world's coordinate system which is
				// defined as a direct orthonormal basis. R is the identity
				// matrix when the device is aligned with the world's coordinate
				// system, that is, when the device's X axis points toward East,
				// the Y axis points to the North Pole and the device is facing
				// the sky. NOTE: the reference coordinate-system used by
				// getOrientation() is different from the world
				// coordinate-system defined for the rotation matrix R and
				// getRotationMatrix().
				float[] r = new float[9];

				// Get the rotation matrix to put our local device coordinates
				// into the world-coordinate system.
				if (SensorManager.getRotationMatrix(r, null, inputAccel,
						inputMag))

				{
					// values[0]: azimuth/yaw, rotation around the Z axis.
					// values[1]: pitch, rotation around the X axis.
					// values[2]: roll, rotation around the Y axis.
					float[] values = new float[3];

					// NOTE: the reference coordinate-system used is different
					// from the world coordinate-system defined for the rotation
					// matrix:
					// X is defined as the vector product Y.Z (It is tangential
					// to the ground at the device's current location and
					// roughly points West). Y is tangential to the ground at
					// the device's current location and points towards the
					// magnetic North Pole. Z points towards the center of the
					// Earth and is perpendicular to the ground.
					SensorManager.getOrientation(r, values);

					// Transfer the rotation matrix to a data structure
					// we can perform matrix algebra with.
					RealMatrix rotation = new Array2DRowRealMatrix(3, 3);

					rotation.setEntry(0, 0, r[0]);
					rotation.setEntry(0, 1, r[1]);
					rotation.setEntry(0, 2, r[2]);
					rotation.setEntry(1, 0, r[3]);
					rotation.setEntry(1, 1, r[4]);
					rotation.setEntry(1, 2, r[5]);
					rotation.setEntry(2, 0, r[6]);
					rotation.setEntry(2, 1, r[7]);
					rotation.setEntry(2, 2, r[8]);

					// Transfer the acceleration to a data structure
					// we can perform matrix algebra with.
					RealVector accelVector = new ArrayRealVector(3);

					accelVector.setEntry(0, inputAccel[0]);
					accelVector.setEntry(1, inputAccel[1]);
					accelVector.setEntry(2, inputAccel[2]);

					// Multiply the acceleration by the rotation matrix
					// to produce the acceleration in the world-coordinate
					// system.
					// Note: the x and y axes will always be equal to zero.
					// The z-axis will be equal to the devices measure of earths
					// gravity.
					RealVector output = rotation.operate(accelVector);

					// Get a steady reading of the gravity of earth as measured
					// by the device regardless of the local device orientation.
					// This is a better value to use than
					// SensorMananger.GRAVITY_EARTH since most devices do
					// not measure the gravity of earth accurately.
					double gravity = output.getEntry(2);
					
					double magnitude = (float) (Math.sqrt(Math.pow(inputAccel[0], 2)
							+ Math.pow(inputAccel[1], 2) + Math.pow(inputAccel[2], 2)) / gravity);

					Log.d(tag, "Magnitude: " + Double.toString(magnitude));

					// The gravity components of the acceleration signal.
					float[] components = new float[3];

					// Find the gravity component of the X-axis
					// = -g*sin(roll);
					components[0] = (float) (-gravity * Math.sin(values[2]));

					// Find the gravity component of the Y-axis
					// = -g*cos(roll)*sin(pith);
					components[1] = (float) (-gravity * Math.cos(values[2]) * Math
							.sin(values[1]));

					// Find the gravity component of the Z-axis
					// = -g*cos(roll)*cos(pitch);
					components[2] = (float) (gravity * Math.cos(values[2]) * Math
							.cos(values[1]));

					float[] tiltAccel = new float[3];

					// Subtract the gravity component of the signal
					// from the input acceleration signal to get the
					// tilt compensated output.
					tiltAccel[0] = inputAccel[0] - components[0];
					tiltAccel[1] = inputAccel[1] - components[1];
					tiltAccel[2] = inputAccel[2] - components[2];

					//Log.d(tag, "Tilt Accel: " + Arrays.toString(tiltAccel));
				}

			}
		}

		this.timestampAccelOld = this.timestampAccel;
		this.timestampMagOld = this.timestampMag;
	}

}
