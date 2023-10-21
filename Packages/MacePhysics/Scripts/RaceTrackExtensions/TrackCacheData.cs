using UnityEngine;

namespace MR.RaceTrackExtensions
{
    public struct TrackCacheData
    {
        public float Z;
        public RacetrackSegment Segment;
        public Vector3 WorldPos;
        public Vector3 WorldForward;
        public Vector3 WorldUp;
        public RacetrackWidening Wide;
        public Matrix4x4 WorldFromSegment;
    }
}