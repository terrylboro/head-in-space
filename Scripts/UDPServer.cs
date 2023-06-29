using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;


public class UDPServer : MonoBehaviour
{
	//UdpClient udpServer;
	string recvStr = "";
	Thread thread;
	static UdpClient udp;
	
	//public float Angle_X, Angle_Y, Angle_Z;
    public float Quatern_X, Quatern_Y, Quatern_Z, Quatern_W = 0;
	
    // Start is called before the first frame update
    void Start()
    { 
		//udpServer = new UdpClient(11000);
		thread = new Thread(new ThreadStart(ThreadMethod));
		thread.Start();
    }

    // Update is called once per frame
    void Update()
    {
		//var remoteEP = new IPEndPoint(IPAddress.Any, 11000); 
		//Debug.Log(remoteEP);
		//var data = udpServer?.Receive(ref remoteEP); // listen on port 11000
		//Debug.Log("receive data from " + remoteEP.ToString());
		//udpServer?.Send(new byte[] { 1 }, 1, remoteEP); // reply back
        
    }
	
	private void ThreadMethod()
	{
		udp = new UdpClient(11000);
		while (true)
		{
			IPEndPoint RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 0);
			byte[] receiveBytes = udp.Receive(ref RemoteIpEndPoint);
			recvStr = Encoding.ASCII.GetString(receiveBytes);
            //Debug.Log(recvStr);
			// perform the rotation using info from returnData
			//Rotation Comd: RXXXXXX
            int Index_X = recvStr.IndexOf('X');
            int Index_Y = recvStr.IndexOf('Y');
            int Index_Z = recvStr.IndexOf('Z');
            int Index_W = recvStr.IndexOf('W');
            int Index_E = recvStr.IndexOf('E');
            Quatern_X = float.Parse(recvStr.Substring(Index_X + 1, (Index_Y - Index_X - 1)));
            Quatern_Y = float.Parse(recvStr.Substring(Index_Y + 1, (Index_Z - Index_Y - 1)));
            Quatern_Z = float.Parse(recvStr.Substring(Index_Z + 1, (Index_W - Index_Z - 1)));
            Quatern_W = float.Parse(recvStr.Substring(Index_W + 1, (Index_E - Index_W - 1)));
		}
	}
}
