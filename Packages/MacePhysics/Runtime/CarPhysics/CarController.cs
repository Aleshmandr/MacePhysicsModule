using UnityEditor;
using UnityEngine;

namespace MR.CarPhysics
{
    public class CarController : MonoBehaviour
    {
        [SerializeField] private Rigidbody rigidBody;
        [SerializeField] private Transform centerOfMass;
        [SerializeField] private WheelJointData[] wheelJoints;
        [SerializeField] private LayerMask mask;
        [SerializeField] private float maxSteerAngle;
        [SerializeField] private float maxSpeed;
        [SerializeField] private AnimationCurve brakeSpeed;
        [SerializeField] private AnimationCurve powerCurve;
        [SerializeField] private float power;
        [SerializeField] private float brakePower;
        private float accelerationInput;
        private float brakeProgress;
        private float brakeDragProgress;

        public Rigidbody Rigidbody => rigidBody;

        public float MaxSpeed => maxSpeed;
        public float Speed { get; private set; }

        public bool IsGrounded { get; private set; }

        private void Awake()
        {
            ResetRigidbody();
        }

        private void Start()
        {
            rigidBody.centerOfMass = centerOfMass.localPosition;
            foreach (var c in GetComponentsInChildren<Collider>())
            {
                c.hasModifiableContacts = true;
            }
        }

        private void ResetRigidbody()
        {
            if (rigidBody != null)
            {
                return;
            }

            rigidBody = GetComponentInParent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            CheckIfGrounded();

            Speed = Vector3.Dot(transform.forward, rigidBody.velocity);
            float normalizedSpeed = Speed / maxSpeed;

            for (int i = 0; i < wheelJoints.Length; i++)
            {
                if (wheelJoints[i].Wheel.IsGrounded)
                {
                    if (wheelJoints[i].IsAccelerating && accelerationInput > 0f)
                    {
                        brakeProgress = 0f;
                        float availableTorque = powerCurve.Evaluate(normalizedSpeed) * power * accelerationInput;
                        var tireTransform = wheelJoints[i].Wheel.transform;
                        Vector3 accelerationDir = tireTransform.forward;
                        rigidBody.AddForceAtPosition(accelerationDir * availableTorque, tireTransform.position, ForceMode.Acceleration);
                    }

                    if (wheelJoints[i].IsBraking && accelerationInput <= 0f)
                    {
                        var tireTransform = wheelJoints[i].Wheel.transform;
                        Vector3 accelerationDir = tireTransform.forward;
                        float brakeValue = brakeSpeed.Evaluate(Speed);
                        float brakeValueInv = 1f - brakeValue;
                        rigidBody.AddForceAtPosition(-accelerationDir * (brakePower * brakeValueInv), tireTransform.position, ForceMode.Acceleration);
                        brakeProgress = brakeValue;
                    }

                    wheelJoints[i].Wheel.SetBrake(brakeProgress);
                }
            }

            if (IsGrounded && rigidBody.velocity.magnitude > maxSpeed)
            {
                rigidBody.velocity = maxSpeed * rigidBody.velocity.normalized;
            }
        }

        private void CheckIfGrounded()
        {
            IsGrounded = true;
            foreach (var wheelJoint in wheelJoints)
            {
                if (wheelJoint.Wheel.IsGrounded)
                {
                    return;
                }
            }

            IsGrounded = false;
        }

        public void SetSteeringAngle(float angle)
        {
            foreach (var wheelJoint in wheelJoints)
            {
                wheelJoint.Wheel.SetSteeringAngle(angle * wheelJoint.Steering);
            }
        }

        public void UpdateAccelerationInput(float accelerationAxis)
        {
            accelerationInput = accelerationAxis;
        }

        public void UpdateSteerInput(float steeringAxis)
        {
            SetSteeringAngle(maxSteerAngle * steeringAxis);
        }

#if UNITY_EDITOR

        private void Reset()
        {
            ResetRigidbody();
            ResetWheels();
        }

        private void ResetWheels()
        {
            if (wheelJoints != null && wheelJoints.Length > 0)
            {
                return;
            }

            var childWheels = GetComponentsInChildren<RaycastWheel>();
            if (childWheels == null || childWheels.Length == 0)
            {
                return;
            }

            wheelJoints = new WheelJointData[childWheels.Length];
            for (int i = 0; i < childWheels.Length; i++)
            {
                wheelJoints[i] = new WheelJointData()
                {
                    Wheel = childWheels[i]
                };
            }

            EditorUtility.SetDirty(this);
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawSphere(rigidBody.centerOfMass, 0.1f);
        }

#endif
    }
}