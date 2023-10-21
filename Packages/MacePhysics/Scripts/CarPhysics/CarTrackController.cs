using MR.Common;
using MR.RaceTrackExtensions;
using Unity.Collections;
using UnityEngine;
using VacuumBreather;

namespace MR.CarPhysics
{
    public class CarTrackController : MonoBehaviour
    {
        [SerializeField] private CarController car;
        [SerializeField] private TrackCache trackCache;
        [SerializeField] private Vector3 rotationPid;
        [SerializeField] private PID steeringPid;
        [SerializeField] private AnimationCurve steeringSideForceFactor;
        [SerializeField] private PID pidX;
        [SerializeField] private PID pidXXX;
        [SerializeField] private PID pidJump;
        [SerializeField] private float onTrackMaxHeight = 1f;
        [SerializeField] private float offTrackGravity;
        [SerializeField] private float onTrackDownForce;
        [SerializeField] private float onTrackDownForceSpeed;
        [SerializeField] private float camP;
        private PidQuaternionController rotationController;
        private TrackCacheData currentTrackPoint;
        private int carBodyId;
        private Vector3 targetWorldTrackPosition;
        private Vector3 trackForward;
        private Vector3 trackUp;
        private Vector3 carForward;
        private float trackControlOffsetX;
        private float flyOverTrackBlending;
        private float jumpStartPoint;
        private bool hasInput;

        private void OnEnable()
        {
            rotationController = new PidQuaternionController(rotationPid.x, rotationPid.y, rotationPid.z);
            carBodyId = car.Rigidbody.GetInstanceID();
            Physics.ContactModifyEvent += PhysicsOnContactModifyEvent;
            Physics.ContactModifyEventCCD += PhysicsOnContactModifyEvent;
        }

        private void OnDisable()
        {
            Physics.ContactModifyEvent -= PhysicsOnContactModifyEvent;
            Physics.ContactModifyEventCCD -= PhysicsOnContactModifyEvent;
        }


        void Update()
        {
            var steeringAxis = Input.GetAxis("Horizontal");
            var accelerationAxis = Input.GetAxis("Vertical");
            car.UpdateAccelerationInput(accelerationAxis);
            if (car.IsGrounded)
            {
                trackControlOffsetX += steeringAxis * Time.deltaTime * 50f;
            }

            hasInput = steeringAxis != 0 || accelerationAxis != 0;
        }

        private void FixedUpdate()
        {
            if (!trackCache.TryGetNearest(car.transform.position, out currentTrackPoint))
            {
                return;
            }

            Vector3 carCacheTrackPosition = currentTrackPoint.WorldFromSegment.inverse.MultiplyPoint(car.transform.position);
            Vector3 carTrackPosition = new Vector3(carCacheTrackPosition.x, carCacheTrackPosition.y, currentTrackPoint.Z);

            float min = -currentTrackPoint.Wide.Left - 5;
            float max = currentTrackPoint.Wide.Right + 5;

            if (hasInput)
            {
                trackControlOffsetX = Mathf.Clamp(trackControlOffsetX, min, max);
            } else
            {
                trackControlOffsetX = Mathf.Clamp(carTrackPosition.x, min, max);
            }

            Vector3 targetCarTrackPosition = new Vector3(trackControlOffsetX, carCacheTrackPosition.y, currentTrackPoint.Z + 10f);
            targetWorldTrackPosition =
                trackCache.Racetrack.CalcWorldPos(targetCarTrackPosition, out Matrix4x4 trackMatrix, out RacetrackSegment segment, out _);

            Vector3 trackUp = trackMatrix.MultiplyVector(Vector3.up);
            Vector3 trackRight = trackMatrix.MultiplyVector(Vector3.right);
            trackForward = trackMatrix.MultiplyVector(Vector3.forward);


            Debug.DrawRay(targetWorldTrackPosition, trackUp * 5f, Color.blue);


            float steeringError = Vector3.Dot(car.transform.right, (targetWorldTrackPosition - car.transform.position).normalized);
            float si = steeringPid.Update(steeringError, Time.fixedDeltaTime);

            bool isOverTrack = carCacheTrackPosition.y > -0.2f;
            bool isFlyOverTrack = !car.IsGrounded && !segment.Curve.IsJump && isOverTrack;

            if (isOverTrack)
            {
                car.UpdateSteerInput(si);
            }

            float forceX = pidX.Update(targetCarTrackPosition.x - carTrackPosition.x, Time.fixedDeltaTime);
            forceX *= steeringSideForceFactor.Evaluate(car.Speed);

            float border = 0.5f;
            float bmin = min + border;
            float bmax = max - border;

            float xError = carTrackPosition.x < bmin ? carTrackPosition.x - bmin : carTrackPosition.x > bmax ? carTrackPosition.x - bmax : 0f;
            xError *= -1f;
            float forceXXX = pidXXX.Update(xError, Time.fixedDeltaTime);
            car.Rigidbody.AddForce(trackRight * forceXXX, ForceMode.Acceleration);

            var targetRotation = Quaternion.LookRotation(trackForward, trackUp);

            Vector3 requiredAngularAcceleration = rotationController.ComputeRequiredAngularAcceleration(car.Rigidbody.rotation,
                targetRotation,
                car.Rigidbody.angularVelocity,
                Time.fixedDeltaTime);

            if (isFlyOverTrack)
            {
                flyOverTrackBlending += Time.fixedDeltaTime * 2f;
                flyOverTrackBlending = Mathf.Clamp01(flyOverTrackBlending);
                float hm = 1f - Mathf.Clamp01((carTrackPosition.y - 1f) / 20f);
                car.Rigidbody.AddTorque(requiredAngularAcceleration * (hm * flyOverTrackBlending), ForceMode.Acceleration);
                car.Rigidbody.AddForce(trackUp * offTrackGravity, ForceMode.Acceleration);
            } else
            {
                flyOverTrackBlending = 0f;
            }


            if (isOverTrack)
            {
                if (segment.Curve.IsJump)
                {
                    if (segment.Curve.BehaviourType == TrackBehaviourType.Fly)
                    {
                        if (jumpStartPoint <= 0f)
                        {
                            jumpStartPoint = carTrackPosition.z;
                        }

                        float progress = (carTrackPosition.z - jumpStartPoint) / segment.Curve.Length;
                        float forceY = pidJump.Update(-carTrackPosition.y + 4f * progress, Time.fixedDeltaTime);

                        Vector3 carVelocityOverTrack = Vector3.ProjectOnPlane(car.Rigidbody.velocity, trackUp);
                        float speedRatio = carVelocityOverTrack.magnitude / car.MaxSpeed;

                        Debug.Log($"sr={speedRatio}");
                        car.Rigidbody.AddForce(Vector3.up * (forceY * speedRatio), ForceMode.Acceleration);
                    }
                } else
                {
                    car.Rigidbody.AddForce(trackRight * forceX, ForceMode.Acceleration);
                    float yd = carTrackPosition.y;

                    if (yd > onTrackMaxHeight)
                    {
                        car.Rigidbody.AddForce(trackUp * offTrackGravity, ForceMode.Acceleration);
                    } else
                    {
                        car.Rigidbody.AddForce(trackUp * Mathf.Lerp(0f, onTrackDownForce, car.Speed / onTrackDownForceSpeed), ForceMode.Acceleration);
                    }

                    jumpStartPoint = 0f;
                }
            }

            carForward = car.transform.forward;
        }

        private void PhysicsOnContactModifyEvent(PhysicsScene scene, NativeArray<ModifiableContactPair> pairs)
        {
            if (!car.IsGrounded)
            {
                return;
            }

            foreach (var contactPair in pairs)
            {
                if (contactPair.bodyInstanceID != carBodyId || contactPair.otherBodyInstanceID != 0)
                {
                    continue;
                }

                for (int i = 0; i < contactPair.contactCount; i++)
                {
                    float roadDirDot = Mathf.Abs(Vector3.Dot(trackForward, contactPair.GetNormal(i)));
                    float carDirDot = Mathf.Abs(Vector3.Dot(carForward, contactPair.GetNormal(i)));

                    if (roadDirDot > 0.6f && carDirDot > 0.6f)
                    {
                        contactPair.IgnoreContact(i);
                    }
                }
            }
        }
    }
}