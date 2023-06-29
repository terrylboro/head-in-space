using UnityEngine;
using System.IO.Ports;
using System.Threading;

public class ArduinoComms : MonoBehaviour
{
	// Having data sent and recieved in a seperate thread to the main game thread stops unity from freezing
    Thread IOThread = new Thread(DataThread);
    private static SerialPort sp; 
    // Stores any data that comes in via the serial port
    private static string incomingMsg = "";

    private static void DataThread()
    {
        // Opens the serial port for reading and writing data
        sp = new SerialPort("COM18", 115200); // Alter the first value to be whatever port the arduino is connected to within the arduino IDE; Alter the second value to be the same as Serial.beign at the start of the arduino program
        sp.Open();
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
    // Start is called before the first frame update
    void Start()
    {
        IOThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
		if(incomingMsg != "")
		{
			Debug.Log(incomingMsg);
		}
        
    }
}
