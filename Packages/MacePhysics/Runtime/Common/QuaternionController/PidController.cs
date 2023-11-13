using System;
using UnityEngine;

namespace MacePhysics.Common
{
    public class PidController
    {
        private const float MaxOutput = 1000f;

        private float integralMax;
        private float integral;

        private float kp;
        private float ki;
        private float kd;

        /// <summary>
        ///     Initializes a new instance of the <see cref="PidController" /> class.
        /// </summary>
        /// <param name="kp">The proportional gain.</param>
        /// <param name="ki">The integral gain.</param>
        /// <param name="kd">The derivative gain.</param>
        /// <exception cref="ArgumentOutOfRangeException">If one of the parameters is negative.</exception>
        public PidController(float kp, float ki, float kd)
        {
            if (kp < 0.0f)
            {
                throw new ArgumentOutOfRangeException(nameof(kp), "kp must be a non-negative number.");
            }

            if (ki < 0.0f)
            {
                throw new ArgumentOutOfRangeException(nameof(ki), "ki must be a non-negative number.");
            }

            if (kd < 0.0f)
            {
                throw new ArgumentOutOfRangeException(nameof(kd), "kd must be a non-negative number.");
            }

            Kp = kp;
            Ki = ki;
            Kd = kd;

            integralMax = MaxOutput / Ki;
        }

        /// <summary>
        ///     Gets or sets the proportional gain.
        /// </summary>
        /// <value>
        ///     The proportional gain.
        /// </value>
        public float Kp
        {
            get => kp;
            set
            {
                if (value < 0f)
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "Kp must be a non-negative number.");
                }

                kp = value;
            }
        }

        /// <summary>
        ///     Gets or sets the integral gain.
        /// </summary>
        /// <value>
        ///     The integral gain.
        /// </value>
        public float Ki
        {
            get => ki;
            set
            {
                if (value < 0f)
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "Ki must be a non-negative number.");
                }

                ki = value;
                integralMax = MaxOutput / Ki;
                integral = Mathf.Clamp(integral, -integralMax, integralMax);
            }
        }

        /// <summary>
        ///     Gets or sets the derivative gain.
        /// </summary>
        /// <value>
        ///     The derivative gain.
        /// </value>
        public float Kd
        {
            get => kd;
            set
            {
                if (value < 0.0f)
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "Kd must be a non-negative number.");
                }

                kd = value;
            }
        }

        /// <summary>
        ///     Computes the corrective output.
        /// </summary>
        /// <param name="error">The current error of the signal.</param>
        /// <param name="delta">The delta of the signal since last frame.</param>
        /// <param name="deltaTime">The delta time.</param>
        /// <returns>The corrective output.</returns>
        public float ComputeOutput(float error, float delta, float deltaTime)
        {
            integral += error * deltaTime;
            integral = Mathf.Clamp(integral, -integralMax, integralMax);

            float derivative = delta / deltaTime;
            float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            output = Mathf.Clamp(output, -MaxOutput, MaxOutput);

            return output;
        }
    }
}