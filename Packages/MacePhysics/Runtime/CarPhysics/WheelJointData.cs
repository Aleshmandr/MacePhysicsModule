namespace MR.CarPhysics
{
    [System.Serializable]
    public struct WheelJointData
    {
        public RaycastWheel Wheel;
        public float Steering;
        public bool IsAccelerating;
        public bool IsBraking;
    }
}