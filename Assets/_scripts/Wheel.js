#pragma strict
var rb 	   : Rigidbody;
var model  : Transform;
var dummy  : GameObject;
var ground : boolean;
var input  : Car;

@Header ("SUSPENSION")
var suspensionTravel : float;
var wheelRadius		 : float;
var springConstant	 : float;
var damperConstant	 : float;
var massMultiplier	 : float;

@Header ("CONSTANTS")
var grip : float;
@Range(-7, 0)
var camber 			  : int;
var a 				  : float[];
var b 				  : float[];
var inertia 		  : float;
var driveTrainInertia : float;
var steerAngle 		  : float;

// PRIVATE VARIABLES //
private var localRotation		 : Quaternion;
private var inverseLocalRotation : Quaternion;
private var groundNormal		 : Vector3;
private var contactPoint		 : Vector3;
private var localVelocity		 : Vector3;
private var wheelVelocity		 : Vector3;
private var roadForce			 : Vector3;
private var slipVelocity		 : float;
private var angularVelocity		 : float;
private var previousCompression  : float;
private var currentCompression	 : float;
private var springVelocity		 : float;
private var springForce			 : float;
private var damperForce			 : float;

private var forward	 : Vector3;
private var right	 : Vector3;
private var maxSlip  : float;
private var maxAngle : float;
private var oldAngle : float;

private var slipRatio : float;
private var slipAngle : float;
private var rotation  : float;

@Header ("OUTPUTS")
var driveTorque 			: float;
var frictionTorque			: float;
var driveFrictionTorque 	: float;
var brakeFrictionTorque 	: float;
var handbrakeFrictionTorque : float;

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
			return -localVelocity.normalized * (Mathf.Abs(unitAngle / p * LateralForceUnit(Fz, p)) + Mathf.Abs(unitSlip / p * LongitudinalForceUnit(Fz, p)));
		} else {
			forward = new Vector3(0, -groundNormal.z, groundNormal.y);
			return Vector3.right * unitAngle / p * LateralForceUnit(Fz, p) + forward * unitSlip / p * LongitudinalForceUnit(Fz, p);
		}
	} else {
		return Vector3.zero;
	}
}
function InitSlipMaxima () {
	var stepSize    	: float = 0.001;
	var testNormalForce : float = 4000;
	var force			: float;
	for (var slip : float = stepSize;; slip += stepSize) {
		var newForce : float = PacejkaFX (testNormalForce, slip);
		if (force < newForce) {
			force = newForce;
		} else {
			maxSlip = slip - stepSize;
			break;
		}
	}
	force = 0;
	for (var angle : float = stepSize;; angle += stepSize) {
		newForce = PacejkaFY (testNormalForce, angle);
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
function SlipRatio () : float {
	var fullSlipVelocity  : float = 4.0;

	var wheelRoadVelocity : float = Vector3.Dot(wheelVelocity, forward);
	if (wheelRoadVelocity == 0)
		return 0;

	var absRoadVelocity   : float = Mathf.Abs(wheelRoadVelocity);
	var damping			  : float = Mathf.Clamp01(absRoadVelocity / fullSlipVelocity);

	var wheelTireVelocity : float = angularVelocity * wheelRadius;
	return (wheelTireVelocity - wheelRoadVelocity) / absRoadVelocity * damping;
}
function SlipAngle () : float {
	var fullAngleVelocity : float = 2.0;

	var wheelMotionDirection : Vector3 = localVelocity;
	wheelMotionDirection.y   = 0;

	if (wheelMotionDirection.sqrMagnitude < Mathf.Epsilon)
		return 0;

	var sinSlipAngle : float = wheelMotionDirection.normalized.x;
	Mathf.Clamp(sinSlipAngle, -1, 1);

	var damping 	 : float = Mathf.Clamp01(localVelocity.magnitude / fullAngleVelocity);

	return -Mathf.Asin(sinSlipAngle) * damping * damping;
}
function RoadForce () : Vector3 {
	var slipRes : int = 100 - Mathf.Round(Mathf.Abs(angularVelocity)) / 10;
	if (slipRes < 1)
		slipRes = 1;
	var invSlipRes : float = 1.0 / slipRes;

	var totalInertia 	  	: float = inertia + driveTrainInertia;
	var driveAngularDelta 	: float = driveTorque * Time.deltaTime * invSlipRes / totalInertia;
	var totalFrictionTorque : float = brakeFrictionTorque * input.brake + handbrakeFrictionTorque * input.handbrake + frictionTorque + driveFrictionTorque;
	var frictionAngularDelta: float = totalFrictionTorque * Time.deltaTime * invSlipRes / totalInertia;

	var totalForce : Vector3 = Vector3.zero;
	var newAngle   : float   = steerAngle * input.steer;
	for(var i : int = 0; i < slipRes; i++) {
		var f : float = i * 1.0 / slipRes;
		localRotation 		 = Quaternion.Euler(0, oldAngle + (newAngle - oldAngle) * f, 0);
		inverseLocalRotation = Quaternion.Inverse(localRotation);
		forward				 = transform.TransformDirection(localRotation * Vector3.forward);
		right				 = transform.TransformDirection(localRotation * Vector3.right);
		right.y 			 = 0;

		slipRatio = SlipRatio();
		slipAngle = SlipAngle();
		var force 	   : Vector3 = invSlipRes * grip * CombinedForces(springForce, slipRatio, slipAngle);
		var worldForce : Vector3 = transform.TransformDirection(localRotation * force);
		angularVelocity -= (force.z * wheelRadius * Time.deltaTime) / totalInertia;
		angularVelocity += driveAngularDelta;
		if (Mathf.Abs(angularVelocity) > frictionAngularDelta) 
			angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
		else 
			angularVelocity  = 0;

		wheelVelocity += worldForce * (1 / rb.mass) * Time.deltaTime * invSlipRes;
		totalForce	  += worldForce;
	}

	var longitudinalSlipVelocity : float = Mathf.Abs(angularVelocity * wheelRadius - Vector3.Dot(wheelVelocity, forward));
	var lateralSlipVelocity 	 : float = Vector3.Dot(wheelVelocity, right);
	slipVelocity = Mathf.Sqrt(longitudinalSlipVelocity * longitudinalSlipVelocity + lateralSlipVelocity * lateralSlipVelocity);

	oldAngle = newAngle;
	return totalForce;
}
function FixedUpdate () {
	var pos : Vector3 = dummy.transform.position;
	var hit : RaycastHit;
	ground = Physics.Raycast(pos, -dummy.transform.up, hit, suspensionTravel + wheelRadius);

	if (ground) {
		groundNormal		= transform.InverseTransformDirection(inverseLocalRotation * hit.normal);
		previousCompression = currentCompression;
		currentCompression  = suspensionTravel - (hit.distance - wheelRadius);
		springVelocity		= (currentCompression - previousCompression) / Time.deltaTime;
		springForce			= springConstant * currentCompression;
		damperForce			= damperConstant * springVelocity;
		contactPoint		= hit.point;
		wheelVelocity 		= rb.GetPointVelocity(pos);
		localVelocity		= transform.InverseTransformDirection(inverseLocalRotation * wheelVelocity);
		roadForce			= RoadForce();

		rb.AddForceAtPosition((dummy.transform.up * (springForce + damperForce)) + roadForce, hit.point);
	} else {
		roadForce = Vector3.zero;
	}

	if (ground)
		model.transform.position = contactPoint + (wheelRadius * transform.parent.up);
	else
		model.transform.position = dummy.transform.position - (suspensionTravel * transform.parent.up);
}
function LateUpdate () {
	var side : float = -Mathf.Sign(transform.localPosition.x);
	rotation += angularVelocity * Time.deltaTime;
	model.transform.localEulerAngles = new Vector3(0, steerAngle * input.steer, side * camber);
	model.Rotate(rotation * Mathf.Rad2Deg, 0, 0);
}