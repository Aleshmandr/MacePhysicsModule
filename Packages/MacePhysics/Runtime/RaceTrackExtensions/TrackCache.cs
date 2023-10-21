using System.Collections.Generic;
using Octree;
using UnityEngine;

namespace MRM.RaceTrackExtensions
{
    public class TrackCache : MonoBehaviour
    {
        //TODO: Add distance lods for optimisation
        [SerializeField] private Racetrack raceTrack;
        [SerializeField] private float step = 0.5f;
        private List<TrackCacheData> alloc;

        public PointOctree<TrackCacheData> CacheData => cacheData;

        private PointOctree<TrackCacheData> cacheData;

        public Racetrack Racetrack => raceTrack;

        private void Awake()
        {
            alloc = new List<TrackCacheData>();
            cacheData = new PointOctree<TrackCacheData>(10f, Vector3.zero, 1f);

            float cacheProgressZ = 0f;
            var path = raceTrack.Path;
            while (true)
            {
                var worldPos = raceTrack.CalcWorldPos(new Vector3(0f, 0f, cacheProgressZ), out Matrix4x4 worldFromSegment, out RacetrackSegment segment,
                    out float segOffset);

                RacetrackWidening w = segment.GetWidening(segOffset);

                cacheData.Add(new TrackCacheData()
                {
                    Z = cacheProgressZ,
                    WorldPos = worldPos,
                    WorldForward = worldFromSegment.MultiplyVector(Vector3.forward),
                    WorldUp = worldFromSegment.MultiplyVector(Vector3.up),
                    WorldFromSegment = worldFromSegment,
                    Segment = segment,
                    Wide = w
                }, worldPos);

                if (cacheProgressZ >= path.TotalLength)
                {
                    break;
                }

                cacheProgressZ += step;
            }

            //foreach(var data in cacheData) {
            //    Debug.DrawRay(data.WorldPos, data.WorldUp, Color.cyan, 5f);
            //}
        }

        public bool TryGetNearest(Vector3 worldPoint, out TrackCacheData nearestPoint)
        {
            nearestPoint = default;

            if (!cacheData.GetNearbyNonAlloc(worldPoint, 2f, alloc) && !cacheData.GetNearbyNonAlloc(worldPoint, 10f, alloc) &&
                !cacheData.GetNearbyNonAlloc(worldPoint, 30, alloc) && !cacheData.GetNearbyNonAlloc(worldPoint, 60, alloc))
            {
                return false;
            }

            float minDistSqr = float.MaxValue;
            foreach (var data in alloc)
            {
                float sqrDist = Vector3.SqrMagnitude(worldPoint - data.WorldPos);
                if (sqrDist < minDistSqr)
                {
                    nearestPoint = data;
                    minDistSqr = sqrDist;
                }
            }

            return true;
        }
    }
}