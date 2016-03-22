#pragma strict
var rb  : Rigidbody;
var com : Transform;

function Start () {
	rb = GetComponent.<Rigidbody>();
}
function Update () {
	rb.centerOfMass = com.transform.localPosition;
}