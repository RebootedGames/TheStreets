#pragma strict
var rb		 : Rigidbody;
var WheelL   : Wheel; 
var WheelR   : Wheel; 
var AntiRoll : float; 

function FixedUpdate () { 
	var hit : RaycastHit; 
	var travelL : float = 1.0; 
	var travelR : float = 1.0;

	var groundedL : boolean = WheelL.ground; 
	if (groundedL) 
		travelL = (-WheelL.transform.InverseTransformPoint(WheelL.contactPoint).y - WheelL.wheelRadius) / WheelL.suspensionTravel; 
	
	var groundedR : boolean = WheelR.ground; 
	if (groundedR) 
		travelR = (-WheelR.transform.InverseTransformPoint(WheelR.contactPoint).y - WheelR.wheelRadius) / WheelR.suspensionTravel; 
	
	var antiRollForce : float = (travelL - travelR) * AntiRoll; 
	if (groundedL) 
		rb.AddForceAtPosition(WheelL.transform.up * antiRollForce, WheelL.transform.position); 
	if (groundedR) 
		rb.AddForceAtPosition(WheelR.transform.up * -antiRollForce, WheelR.transform.position); 
}