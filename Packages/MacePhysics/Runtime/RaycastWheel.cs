using UnityEditor;
using UnityEngine;

namespace MacePhysics
{
    public class RaycastWheel : MonoBehaviour
    {
        [SerializeField] private LayerMask mask;
        [SerializeField] private float radius;
        [SerializeField] private float friction;
        [SerializeField] private AnimationCurve slipCurve;
        [SerializeField] private float springForce;
        [SerializeField] private float springDamping;
        [SerializeField] private float restDistance;
        [SerializeField] private float staticFrictionFactor;
        [SerializeField] private Rigidbody carRigidbody;
        [SerializeField] private Transform meshRoot;
        private Vector3 wheelLocalPosition;
        private RaycastHit hitInfo;
        private float offset;
        private float brakeMultiplier;

        public bool IsGrounded { get; private set; }

        public Vector3 Velocity { get; private set; }

        private void Awake()
        {
            ValidateCarRigidbody();
            SetBrake(1f);
        }

        public void SetSteeringAngle(float angle)
        {
            transform.localRotation = Quaternion.AngleAxis(angle, Vector3.up);
        }

        public void SetBrake(float brake)
        {
            brakeMultiplier = Mathf.Clamp01(brake);
        }

        private void ValidateCarRigidbody()
        {
            if (carRigidbody == null)
            {
                carRigidbody = GetComponentInParent<Rigidbody>();
            }
        }

        private void Update()
        {
            meshRoot.localPosition = wheelLocalPosition;
            if (IsGrounded)
            {
                Velocity = transform.InverseTransformVector(carRigidbody.GetPointVelocity(hitInfo.point));
            } else
            {
                Velocity = transform.InverseTransformVector(carRigidbody.velocity);
            }

            float wheelRotationSpeed = -Mathf.Sign(transform.localPosition.x) * (Velocity.z * Mathf.Rad2Deg) / radius;
            meshRoot.rotation *= Quaternion.Euler(wheelRotationSpeed * Time.deltaTime, 0f, 0f);
        }

        private void FixedUpdate()
        {
            float rayLength = restDistance + radius;
            Ray ray = new Ray(transform.position, -transform.up);
            Debug.DrawRay(ray.origin, ray.direction * rayLength);
            IsGrounded = Physics.Raycast(ray, out hitInfo, rayLength, mask);
            if (IsGrounded)
            {
                Vector3 tireWorldVelocity = carRigidbody.GetPointVelocity(hitInfo.point);
                float tireWorldSpeed = tireWorldVelocity.magnitude;

                // Suspension force
                Vector3 springDir = transform.up;
                offset = restDistance - hitInfo.distance + radius;
                float vel = Vector3.Dot(springDir, tireWorldVelocity);
                float force = offset * springForce - vel * springDamping;
                Vector3 suspensionForce = springDir * force;
                carRigidbody.AddForceAtPosition(suspensionForce, hitInfo.point, ForceMode.Acceleration);

                // Grip force
                if (tireWorldSpeed > 0f)
                {
                    Vector3 sideGripDirection = brakeMultiplier <= 0f ? transform.right : carRigidbody.transform.right;
                    float steeringVelocity = Vector3.Dot(sideGripDirection, tireWorldVelocity);
                    float slipValue = steeringVelocity / tireWorldSpeed;
                    float desiredVelocityChange = -steeringVelocity * slipCurve.Evaluate(Mathf.Abs(slipValue));
                    float desiredAcceleration = desiredVelocityChange / Time.fixedDeltaTime;
                    Vector3 dynamicGripForce = sideGripDirection * (desiredAcceleration * friction);
                    carRigidbody.AddForceAtPosition(dynamicGripForce, hitInfo.point);
                }

                if (brakeMultiplier > 0f)
                {
                    float gravityDot = Vector3.Dot(suspensionForce.normalized, Vector3.up);
                    float frictionMultiplier = Mathf.InverseLerp(0f, 1f - staticFrictionFactor, gravityDot);
                    Vector3 staticGripForce = Vector3.Lerp(Vector3.zero, -Vector3.ProjectOnPlane(suspensionForce, Vector3.up), frictionMultiplier);
                    carRigidbody.AddForceAtPosition(staticGripForce * brakeMultiplier, hitInfo.point, ForceMode.Acceleration);
                    carRigidbody.AddForceAtPosition(-tireWorldVelocity * (0.25f * brakeMultiplier * frictionMultiplier), hitInfo.point, ForceMode.Acceleration);
                }
            } else
            {
                offset = 0f;
            }

            wheelLocalPosition = new Vector3(0f, offset - restDistance, 0f);
        }

#if UNITY_EDITOR

        private void OnDrawGizmos()
        {
            Handles.color = Gizmos.color = new Color(IsGrounded ? 1f : 0f, 1f, 0f, 1f);
            Handles.matrix = Gizmos.matrix = transform.localToWorldMatrix;
            Handles.DrawWireDisc(Vector3.down * (restDistance - offset), Vector3.right, radius);
            Handles.DrawLine(Vector3.zero, Vector3.down * restDistance);
        }

        private void OnDrawGizmosSelected()
        {
            Color color = IsGrounded ? new Color(1f, 0.5f, 0.5f, 0.3f) : new Color(0f, 1f, 0f, 0.3f);
            Handles.color = Gizmos.color = color;
            Handles.matrix = Gizmos.matrix = transform.localToWorldMatrix;
            Handles.DrawSolidDisc(wheelLocalPosition, Vector3.right, radius);
        }

        private void Reset()
        {
            ValidateCarRigidbody();
        }

        private void OnValidate()
        {
            if (Application.isPlaying)
            {
                return;
            }

            wheelLocalPosition = Vector3.down * restDistance;
            meshRoot.localPosition = wheelLocalPosition;
        }

#endif
    }
}