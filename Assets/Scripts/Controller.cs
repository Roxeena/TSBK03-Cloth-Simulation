using UnityEngine;

public class Controller : MonoBehaviour {

    public float speed = 800.0f;
    public float sensitivity = 3.0f;
    public float flightSpeed = 2.0f;

    private float lookX, lookY;
    private float moveFB, moveLR, fly;
    private Camera cam;
    private bool stop = false;

    // Use this for initialization
	void Start () {
        cam = Camera.main;
        fly = 0.0f;
	}
	
	// Update is called once per frame
	void Update () {

        //Turn off controller when hit escape
        if (Input.GetKey(KeyCode.Escape))
        {
            Application.Quit();
            return;
        }
        if (Input.GetKeyDown(KeyCode.Q))
        {
            stop = (stop) ? false : true;
        }
        if (stop)
            return;
            

        //Movement
        moveFB = Input.GetAxis("Vertical") * speed * Time.deltaTime;
        moveLR = Input.GetAxis("Horizontal") * speed * Time.deltaTime;

        //Fly or decend
        if (Input.GetKey(KeyCode.Space))
            fly = flightSpeed * Time.deltaTime;

        else if (Input.GetKey(KeyCode.LeftShift))
            fly = -flightSpeed * Time.deltaTime;

        else
            fly = 0.0f;

        //Camera rotation
        lookX = Input.GetAxis("Mouse X") * sensitivity;
        lookY -= Input.GetAxis("Mouse Y") * sensitivity;
        lookY = Mathf.Clamp(lookY, -60f, 60f);

        //Apply rotation. Tilt head and rotate body
        if (cam)
        {
            transform.Rotate(0.0f, lookX, 0.0f);
            cam.transform.localRotation = Quaternion.Euler(lookY, 0.0f, 0.0f);
        }
        else
            Debug.LogError("No main camera set!");

        //Apply movement
        transform.Translate(new Vector3(moveLR, fly, moveFB));
	}
}
