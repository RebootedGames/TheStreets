#pragma strict

var carTransform	: Transform;
var carObject		: Transform;
var distanceFromCar	: float;
var heightFromCar	: float;
var rotationDamping	: float;
var heightDamping	: float;
var zoomRatio		: float;
var defaultFOV		: float;

private var rotationVector : Vector3;

function LateUpdate () {
	var wantedAngle     = carTransform.eulerAngles.y;
	var wantedHeight    = carTransform.position.y + heightFromCar;
	var myAngle		    = transform.eulerAngles.y;
	var myHeight	    = transform.position.y;
	
	myAngle			    = Mathf.LerpAngle  (myAngle, wantedAngle, rotationDamping * Time.deltaTime);
	myHeight		    = Mathf.Lerp (myHeight, wantedHeight, heightDamping * Time.deltaTime);
	
	var currentRotation = Quaternion.Euler (0, myAngle, 0);
	
	transform.position  = carTransform.position;
	transform.position -= currentRotation * Vector3.forward * distanceFromCar;
	transform.position.y= myHeight;
	transform.LookAt (carTransform);
}

function FixedUpdate () {
	var localVelocity = carObject.InverseTransformDirection (carObject.GetComponent.<Rigidbody>().velocity);
	if (localVelocity.z < -0.5)
		rotationVector.y = carTransform.eulerAngles.y + 180;
	else
		rotationVector.y = carTransform.eulerAngles.y;
		
	var acc = carObject.GetComponent.<Rigidbody>().velocity.magnitude;
	GetComponent.<Camera>().fieldOfView = defaultFOV + acc * zoomRatio * Time.deltaTime;
}