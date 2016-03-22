#pragma strict
var rb 	   : Rigidbody;
var model  : Transform;
var dummy  : GameObject;
var ground : boolean;

@Header ("SUSPENSION")
var suspensionTravel : float;
var wheelRadius		 : float;
var springConstant	 : float;
var damperConstant	 : float;
var massMultiplier	 : float;

@Header ("CONSTANTS")
@Range(-7, 0)
var camber : int;
var a : float[];
var b : float[];

// PRIVATE VARIABLES //
private var contactPoint		: Vector3;
private var localVelocity		: Vector3;
private var previousCompression : float;
private var currentCompression	: float;
private var springVelocity		: float;
private var springForce			: float;
private var damperForce			: float;

private var maxSlip  : float;
private var maxAngle : float;
private var newForce : float;

function PacejkaFX (Fz : float, slip : float) : float {
	Fz   *= 0.001; // convert to kN
	slip *= 100;   // convert to %

	var uP : float = b[1] * Fz + b[2];
	var D  : float = uP * Fz;
	var B  : float = ((b[3] * Fz + b[4]) * Mathf.Exp(-b[5] * Fz)) / (b[0] * uP);
	var S  : float = slip + b[9] * Fz + b[10];
	var E  : float = b[6] * Fz * Fz + b[7] * Fz + b[8];

	var Fx : float = D * Mathf.Sin(b[0] * Mathf.Atan(S * B + E * (Mathf.Atan(S * B) - S * B)));
	return Fx;
}
function PacejkaFY (Fz : float, angle : float) : float {
	Fz    *= 0.001; 				 // convert to kN
	angle *= (360 / (2 * Mathf.PI)); // convert to degrees

	var uP : float = a[1] * Fz + a[2];
	var D  : float = uP * Fz;
	var B  : float = (a[3] * Mathf.Sin(2 * Mathf.Atan(Fz / a[4]))) / (a[0] * uP * Fz);
	var S  : float = angle + a[9] * Fz + a[10];
	var E  : float = a[6] * Fz + a[7];
	var Sv : float = a[12] * Fz + a[13];

	var Fy : float = D * Mathf.Sin(a[0] * Mathf.Atan(S * B + E * (Mathf.Atan(S * B) - S * B))) + Sv;
	return Fy;
}
function LongitudinalForceUnit (Fz : float, slip : float) : float {
	return PacejkaFX (Fz, slip * maxSlip);
}
function LateralForceUnit (Fz : float, angle : float) : float {
	return PacejkaFY (Fz, angle * maxAngle);
}
function CombinedForces (Fz : float, slip : float, angle : float) : Vector3 {
	var unitSlip  : float = slip / maxSlip;
	var unitAngle : float = angle / maxAngle;
	var p		  : float = Mathf.Sqrt(unitSlip*unitSlip + unitAngle*unitAngle);
	if (p > Mathf.Epsilon) {
		if (slip < -0.8) {

		} else {
			var forward : Vector3 = new Vector3(0, -groundNormal.z, groundNormal.y);
			return Vector3.right * unitAngle / p * LateralForceUnit(Fz, p) + forward * unitSlip / p * LongitudinalForceUnit(Fz, p);
		}
	} else {
		return Vector3.zero;
	}
}
function InitSlipMaxima () {
	var stepSize    : float = 0.001;
	var normalForce : float = 4000;
	var force		: float;
	for (var slip : float = stepSize;; slip += stepSize) {
		newForce = PacejkaFX (normalForce, slip);
		if (force < newForce) {
			force = newForce;
		} else {
			maxSlip = slip - stepSize;
			break;
		}
	}
	force = 0;
	for (var angle : float = stepSize;; angle += stepSize) {
		newForce = PacejkaFY (normalForce, angle);
		if (force < newForce) {
			force = newForce;
		} else {
			maxAngle = angle - stepSize;
			break;
		}
	}
}

function Start () {
	rb    = transform.parent.GetComponent.<Rigidbody>();
	dummy = new GameObject("dummy." + gameObject.name);

	dummy.transform.position = transform.position;
	dummy.transform.parent   = transform.parent;

	springConstant = rb.mass * -Physics.gravity.y * massMultiplier;

	InitSlipMaxima();
}
function FixedUpdate () {
	var hit : RaycastHit;
	ground = Physics.Raycast(dummy.transform.position, -dummy.transform.up, hit, suspensionTravel + wheelRadius);

	if (ground) {
		previousCompression = currentCompression;
		currentCompression  = suspensionTravel - (hit.distance - wheelRadius);
		springVelocity		= (currentCompression - previousCompression) / Time.deltaTime;
		springForce			= springConstant * currentCompression;
		damperForce			= damperConstant * springVelocity;
		contactPoint		= hit.point;

		rb.AddForceAtPosition(dummy.transform.up * (springForce + damperForce), hit.point);
	}
}
function LateUpdate () {
	if (ground)
		model.transform.position = contactPoint + (wheelRadius * transform.parent.up);
	else
		model.transform.position = dummy.transform.position - (suspensionTravel * transform.parent.up);

	var side : float = -Mathf.Sign(transform.localPosition.x);
	transform.localEulerAngles = new Vector3(0, 0, side * camber);
}