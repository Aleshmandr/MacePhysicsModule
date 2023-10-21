using System;
using UnityEngine;

namespace MR.Common
{
    [Serializable]
    public class PID
    {
        public float Kp;
        public float Kd;

        [SerializeField] private float ki;
        [SerializeField] private float kiThresh;
        [SerializeField] private float maxAmplitude;
        private float prevError;
        private float integral;

        public float Ki
        {
            get { return ki; }
            set
            {
                if (Math.Abs(ki - value) > float.Epsilon)
                {
                    integral = 0;
                }

                ki = value;
            }
        }

        public PID(float kp, float ki, float kd)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        public float Update(float error, float deltaTime)
        {
            if (Mathf.Abs(error) <= kiThresh)
            {
                integral = 0f;
            } else
            {
                integral += error * deltaTime;
            }

            float derivative = (error - prevError) / deltaTime;
            prevError = error;
            float rawResult = error * Kp + integral * Ki + derivative * Kd;
            float rawAmplitude = Mathf.Abs(rawResult);
            if (maxAmplitude > 0f && rawAmplitude > maxAmplitude)
            {
                return Mathf.Sign(rawResult) * Mathf.Min(rawAmplitude, maxAmplitude);
            }

            return rawResult;
        }

        public void Reset()
        {
            prevError = 0f;
            integral = 0f;
        }
    }
}