using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

public class SerialHeadControl : MonoBehaviour
{
	// Having data sent and recieved in a seperate thread to the main game thread stops unity from freezing
    Thread IOThread = new Thread(DataThread);
    private static SerialPort sp; 
    private static string incomingMsg = "";
	private AHRS.MadgwickAHRS madgwickAHRS;
	private AHRS.MahonyAHRS mahonyAHRS;
	private bool filterName = false; // True for Madgwick, false for Mahony
	public float AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY, MagZ;
	private int frameNum, deltaT;
	public float Quatern_X, Quatern_Y, Quatern_Z, Quatern_W = 0;
	public float roll, pitch, yaw;

    private static void DataThread()
    {
        // Opens the serial port for reading and writing data
        sp = new SerialPort("COM18", 115200); // Alter the first value to be whatever port the arduino is connected to within the arduino IDE; Alter the second value to be the same as Serial.beign at the start of the arduino program
        sp.Open();

        // Every 200ms, it checks if there is a message stores in the output buffer string to be sent to the arduino,
        // Then recieves any data being sent to the project via the arduino 
        while(true)
        {
			incomingMsg = sp.ReadLine();
            Thread.Sleep(1);
        }
    }
	
	private void OnDestroy()
    {
        // Closes the thread and serial port when the game ends
        IOThread.Abort();
        sp.Close();
    }
	
	private void fillDataBuffer(string[] values)
	{
		//frameNum = values[0];
		//deltaT = values[1];
		// it seems that Unity Y and Z axes are flipped
		AccX = float.Parse(values[2]);
		AccY = float.Parse(values[3]);
		AccZ = float.Parse(values[4]);// - 9.6f;
		// are X and Y angles swapped?
		GyrX = float.Parse(values[5]);
		GyrY = float.Parse(values[6]);
		GyrZ = float.Parse(values[7]);
		MagX = float.Parse(values[8]);
		MagY = float.Parse(values[9]);
		MagZ = float.Parse(values[10]);
	}

    // Start is called before the first frame update
    void Start()
    {
		// create the object which holds the algorithm
		// static AHRS.MadgwickAHRS AHRS = new AHRS.MadgwickAHRS(1f / 104f, 0.1f); // rate, beta
        //AHRS = new AHRS.MadgwickAHRS(1f / 104f, 0.75f) if filterName else new AHRS.MahonyAHRS(1f / 104f, 5f); // rate, kp
		//AHRS = filterName ? new AHRS.MadgwickAHRS(1f / 104f, 0.75f) : new AHRS.MahonyAHRS(1f / 104f, 5f); // rate, kp
		if (filterName)
		{
			Debug.Log("Using Madgwick filter");
			madgwickAHRS = new AHRS.MadgwickAHRS(1f / 104f, 0.8f);
		} else
		{
			Debug.Log("Using Mahony filter");
			mahonyAHRS = new AHRS.MahonyAHRS(1f / 104f, 5f);
		}
		IOThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
        if(incomingMsg != "")
		{
			Debug.Log(incomingMsg);
			string[] values = incomingMsg.Split(',');
			fillDataBuffer(values);
			if (filterName)
			{
				madgwickAHRS.Update(Mathf.Deg2Rad * GyrX, Mathf.Deg2Rad * GyrY, Mathf.Deg2Rad * GyrZ, Mathf.Deg2Rad * AccX, Mathf.Deg2Rad * AccY, Mathf.Deg2Rad * AccZ, Mathf.Deg2Rad * MagX, Mathf.Deg2Rad * MagY, Mathf.Deg2Rad * MagZ);
				// 6-axis
				//madgwickAHRS.Update(Mathf.Deg2Rad * GyrX, Mathf.Deg2Rad * GyrY, Mathf.Deg2Rad * GyrZ, Mathf.Deg2Rad * AccX, Mathf.Deg2Rad * AccY, Mathf.Deg2Rad * AccZ);
				Quatern_X = madgwickAHRS.Quaternion[0];
				Quatern_Y = madgwickAHRS.Quaternion[1];
				Quatern_Z = madgwickAHRS.Quaternion[2];
				Quatern_W = madgwickAHRS.Quaternion[3];
			} else
			{
				mahonyAHRS.Update(Mathf.Deg2Rad * GyrX, Mathf.Deg2Rad * GyrY, Mathf.Deg2Rad * GyrZ, Mathf.Deg2Rad * AccX, Mathf.Deg2Rad * AccY, Mathf.Deg2Rad * AccZ, Mathf.Deg2Rad * MagX, Mathf.Deg2Rad * MagY, Mathf.Deg2Rad * MagZ);
				// 6-axis
				//mahonyAHRS.Update(Mathf.Deg2Rad * GyrX, Mathf.Deg2Rad * GyrY, Mathf.Deg2Rad * GyrZ, Mathf.Deg2Rad * AccX, Mathf.Deg2Rad * AccY, Mathf.Deg2Rad * AccZ);
				Quatern_X = mahonyAHRS.Quaternion[0];
				Quatern_Y = mahonyAHRS.Quaternion[1];
				Quatern_Z = mahonyAHRS.Quaternion[2];
				Quatern_W = mahonyAHRS.Quaternion[3];
			}
			
			//Quatern_W = -AHRS.Quaternion[3]; // allow for Unity's different convention
			
			// print out the orientation as Euler angles
			Quaternion q = new Quaternion(Quatern_X, Quatern_Y, Quatern_Z, Quatern_W);
			var qEuler = q.eulerAngles;
			roll = qEuler.x;
			pitch = qEuler.y;
			yaw = qEuler.z;
			//Debug.Log(qEuler.x);
			//Debug.Log(qEuler.y);
			//Debug.Log(qEuler.z);
			
			//Quatern_X = 1;
			//Quatern_Y = 1;
			//Quatern_Z = 1;
			//Quatern_W = 1;
		}
    }

}
