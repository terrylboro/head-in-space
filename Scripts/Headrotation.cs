using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Headrotation : MonoBehaviour
{
    //public float Rota_X, Rota_Y, Rota_Z = 0;
    public float Quar_X, Quar_Y, Quar_Z, Quar_W = 0;
	public float roll, pitch, yaw = 0;
	public Transform t; // to control the rotation of the head
    //float ErrX = 0;
    //float ErrY = -90;
    //float ErrZ = -102.426;
    // Start is called before the first frame update
    void Start()
    {
        //this.transform.rotation = Quaternion.Euler(0,0,0);
    }

    // Update is called once per frame
    void Update()
    {
        //try {
          //  Quar_X = this.GetComponent<UDPServer>().Quatern_X;
        //}       
        //catch (NullReferenceException ex) {
         //   Debug.Log("UDPServer was not set in the inspector");
        //}
		Quar_X = this.GetComponent<SerialHeadControl>().Quatern_X;
        Quar_Y = this.GetComponent<SerialHeadControl>().Quatern_Y;
        Quar_Z = this.GetComponent<SerialHeadControl>().Quatern_Z;
        Quar_W = this.GetComponent<SerialHeadControl>().Quatern_W;
		roll = this.GetComponent<SerialHeadControl>().roll;
		pitch = this.GetComponent<SerialHeadControl>().pitch;
		yaw = this.GetComponent<SerialHeadControl>().yaw;
		//Debug.Log("Updated quars");
        //Quaternion rotation1 = Quaternion.AngleAxis(Quar_W, new Vector3(Quar_X, Quar_Y, Quar_Z));
        //Quaternion rotation1 = new Quaternion(Quar_W, Quar_Y, Quar_Z, Quar_X);
		Quaternion rotation1 = Quaternion.Euler(yaw, roll, -pitch); // yaw, roll, -pitch works when connected on side with XYZ printed
		//Quaternion rotation1 = Quaternion.Euler(yaw, roll, pitch);
		//Quaternion rotation1 = new Quaternion(Quar_X, Quar_Y, Quar_Z, Quar_W);
		Quaternion rotation2 = Quaternion.Euler(0, -90, -102);
        //this.transform.rotation = rotation1 * rotation2;
		this.transform.rotation = rotation1 ;//* Quaternion.Euler(0,0,0);
		//t.transform.rotation = rotation1 * rotation2;
        //this.transform.rotation = (Quaternion.AngleAxis(Quar_W, new Vector3(Quar_X, Quar_Y, Quar_Z)) * Quaternion.Euler(0, -90, -102.426));

        //float x = 1;
        //Rota_X = this.GetComponent<UDPReceive>().Angle_X;
        //Rota_Y = this.GetComponent<UDPReceive>().Angle_Y;
        //Rota_Z = this.GetComponent<UDPReceive>().Angle_Z;
        //this.transform.eulerAngles = new Vector3(Rota_X, Rota_Y, Rota_Z);

        //print("Head rotation, Rota_X: " + Rota_X + " Rota_Y: " + Rota_Y + " Rota_Z: " + Rota_Z);
        //this.transform.Rotate(Vector3.right * Rota_X, Space.Self);
        //this.transform.Rotate(Vector3.up * Rota_Y, Space.Self);
        //this.transform.Rotate(Vector3.forward * Rota_Z, Space.Self);
        //this.transform.Rotate(Rota_X, Rota_Y, Rota_Z, Space.World);


    }
}
