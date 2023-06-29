using System.Net.Sockets;
using System.Net;
using System;
using System.Text;
using System.Collections;

using UnityEngine;
 
public class UDPClient : MonoBehaviour
{
    IEnumerator Start()
    {
        yield return new WaitForSeconds(1);
        Send("test test test");
    }
    private void Send(string msg)
    {
        Socket s = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
 
        IPAddress targetAddress = IPAddress.Parse("127.0.0.1");
 
        byte[] sendbuf = Encoding.ASCII.GetBytes(msg);
        IPEndPoint ep = new IPEndPoint(targetAddress, 11000);
 
        s.SendTo(sendbuf, ep); s.Close();
 
        print("Message sent");
    }
}
