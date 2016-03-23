#pragma strict
var rb  : Rigidbody;
var com : Transform;

@Header ("WHEELS")
var flWheel : Wheel;
var frWheel : Wheel;
var rlWheel : Wheel;
var rrWheel : Wheel;

@Header ("CAR SPECIFICATIONS")
var width 	   : float;
var wheelBase  : float;
var turnCircle : float;

@Header ("INPUTS")
var steer     : float;
var throttle  : float;
var brake     : float;
var handbrake : float;

function Start () {
	rb = GetComponent.<Rigidbody>();
}
function Update () {
	rb.centerOfMass = com.transform.localPosition;

	throttle = Input.GetAxis("Throttle");
	steer    = Input.GetAxis("Steer");
	brake    = Input.GetAxis("Brake");

	rlWheel.driveTorque = 1000 * throttle;
	rrWheel.driveTorque = 1000 * throttle;

	flWheel.steerAngle = Mathf.Atan(wheelBase / (turnCircle - width)) * Mathf.Rad2Deg;
	frWheel.steerAngle = Mathf.Atan(wheelBase / (turnCircle - width)) * Mathf.Rad2Deg;
}