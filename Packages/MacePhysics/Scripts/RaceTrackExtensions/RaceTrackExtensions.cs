using UnityEngine;

namespace MR.RaceTrackExtensions
{
    public static class RaceTrackExtensions
    {
        public static Vector3 CalcWorldPos(this Racetrack racetrack, Vector3 position, out Matrix4x4 worldFromSegment, out RacetrackSegment segment,
            out float segmentZOffset)
        {
            // Convert z to segment and offset
            float distance = position.z % racetrack.Path.TotalLength;
            segment = racetrack.Path.GetSegmentAndOffset(distance, out segmentZOffset);

            // Get transformation for segment
            Matrix4x4 trackFromSegment = segment.GetSegmentToTrack(segmentZOffset);
            Matrix4x4 worldFromTrack = racetrack.transform.localToWorldMatrix;
            worldFromSegment = worldFromTrack * trackFromSegment;

            // Segment space position and orientation
            Vector3 segPos = new Vector3(position.x, position.y, segmentZOffset);

            // Convert to world space
            return worldFromSegment.MultiplyPoint(segPos);
        }
    }
}