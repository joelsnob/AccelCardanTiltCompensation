package com.kircherelectronics.androidtiltcompensation;

import android.os.Bundle;
import android.app.Activity;
import android.view.Menu;

public class TiltCompensationActivity extends Activity
{
	private float timestampAccel;
	private float timestampAccelOld;

	private float timestampMag;
	private float timestampMagOld;
	
	// Raw accelerometer data
	private float[] inputAccel = new float[3];
	private float[] inputMag = new float[3];
	

	@Override
	protected void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_tilt_compensation);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu)
	{
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_tilt_compensation, menu);
		return true;
	}

}
