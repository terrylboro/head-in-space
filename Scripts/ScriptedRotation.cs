using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScriptedRotation : MonoBehaviour
{
	public int frameCount = 0;
	public int y_angle = 10;
	public Vector3 newPosition;
	public Transform transform;
    // Start is called before the first frame update
    void Start()
    {
		Vector3 newPosition = new Vector3(0, y_angle, 0);
		transform.eulerAngles = newPosition;
    }

    // Update is called once per frame
    void Update()
    {
		if (frameCount % 300 == 0)
		{
			y_angle += 10;
			newPosition = new Vector3(0, 0, y_angle);
			transform.eulerAngles = newPosition;
		}
		frameCount++;
    }
}
