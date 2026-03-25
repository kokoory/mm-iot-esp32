/*
 * AHRS - Extended Kalman Filter Implementation
 *
 * 7-state EKF: quaternion (4) + gyro bias (3)
 *
 * Prediction step:
 *   q_new = q + 0.5 * q ⊗ (gyro - bias) * dt
 *   bias_new = bias  (random walk)
 *
 * Measurement updates:
 *   Accelerometer: corrects roll/pitch using gravity vector
 *   Magnetometer:  corrects yaw using Earth's magnetic field
 *
 * Uses a simplified 6x6 error-state covariance (3 attitude + 3 bias)
 * for computational efficiency on ESP32-P4.
 */

#include "ahrs.h"
#include "../common/math_utils.h"
#include <math.h>
#include <string.h>

/* Error state dimension: 3 (attitude error) + 3 (bias error) = 6 */
#define ERR_DIM 6

/* Matrix indexing helper for 6x6 */
#define P6(r,c) ((r)*ERR_DIM + (c))

/* ── Helper: 6x6 matrix operations ─────────────────────────── */

static void mat6_zero(float m[36])
{
    memset(m, 0, 36 * sizeof(float));
}

static void mat6_identity(float m[36])
{
    mat6_zero(m);
    for (int i = 0; i < ERR_DIM; i++) {
        m[P6(i,i)] = 1.0f;
    }
}

/* C = A * B (6x6) */
static void mat6_mul(const float A[36], const float B[36], float C[36])
{
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < ERR_DIM; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++) {
                sum += A[P6(i,k)] * B[P6(k,j)];
            }
            C[P6(i,j)] = sum;
        }
    }
}

/* C = A + B (6x6) */
static void mat6_add(const float A[36], const float B[36], float C[36])
{
    for (int i = 0; i < 36; i++) {
        C[i] = A[i] + B[i];
    }
}

/* B = A^T (6x6) */
static void mat6_transpose(const float A[36], float B[36])
{
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < ERR_DIM; j++) {
            B[P6(i,j)] = A[P6(j,i)];
        }
    }
}

/* ── Rotation matrix from quaternion ───────────────────────── */

static void quat_to_dcm(float q0, float q1, float q2, float q3, float R[9])
{
    float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
    float q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q2 = q1*q2, q1q3 = q1*q3, q2q3 = q2*q3;

    R[0] = q0q0 + q1q1 - q2q2 - q3q3;
    R[1] = 2.0f*(q1q2 - q0q3);
    R[2] = 2.0f*(q1q3 + q0q2);
    R[3] = 2.0f*(q1q2 + q0q3);
    R[4] = q0q0 - q1q1 + q2q2 - q3q3;
    R[5] = 2.0f*(q2q3 - q0q1);
    R[6] = 2.0f*(q1q3 - q0q2);
    R[7] = 2.0f*(q2q3 + q0q1);
    R[8] = q0q0 - q1q1 - q2q2 + q3q3;
}

/* Skew-symmetric matrix [v]x for cross product */
static void skew(const float v[3], float S[9])
{
    S[0] =  0.0f; S[1] = -v[2]; S[2] =  v[1];
    S[3] =  v[2]; S[4] =  0.0f; S[5] = -v[0];
    S[6] = -v[1]; S[7] =  v[0]; S[8] =  0.0f;
}

/* ── Quaternion helpers ────────────────────────────────────── */

static void quat_normalize(float *q0, float *q1, float *q2, float *q3)
{
    float norm = sqrtf(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    if (norm > 1e-8f) {
        float inv = 1.0f / norm;
        *q0 *= inv; *q1 *= inv; *q2 *= inv; *q3 *= inv;
    }
}

/* Small angle quaternion from rotation vector (first-order approximation) */
static void small_angle_quat(const float dtheta[3], float dq[4])
{
    float angle = sqrtf(dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2]);
    if (angle < 1e-8f) {
        dq[0] = 1.0f; dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = 0.0f;
    } else {
        float ha = 0.5f * angle;
        float s = sinf(ha) / angle;
        dq[0] = cosf(ha);
        dq[1] = dtheta[0] * s;
        dq[2] = dtheta[1] * s;
        dq[3] = dtheta[2] * s;
    }
}

/* Quaternion multiplication: r = p * q */
static void quat_mul(const float p[4], const float q[4], float r[4])
{
    r[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    r[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    r[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    r[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
}

/* ── Public API ────────────────────────────────────────────── */

void ahrs_init(ahrs_t *ahrs, float beta)
{
    memset(ahrs, 0, sizeof(ahrs_t));
    ahrs->q0 = 1.0f;
    ahrs->beta = beta;

    /* Default noise parameters */
    ahrs->gyro_noise      = 0.01f;    /* rad/s */
    ahrs->gyro_bias_noise = 0.001f;   /* rad/s^2 */
    ahrs->accel_noise     = 0.5f;     /* m/s^2 */
    ahrs->mag_noise       = 0.3f;     /* normalized */

    /* Initialize 7x7 P matrix (stored but we use 6x6 error-state internally) */
    /* Initialize diagonal of the full P storage */
    memset(ahrs->P, 0, sizeof(ahrs->P));
    /* Attitude uncertainty: ~10 degrees */
    float att_var = DEG_TO_RAD(10.0f);
    att_var *= att_var;
    /* Bias uncertainty: ~1 deg/s */
    float bias_var = DEG_TO_RAD(1.0f);
    bias_var *= bias_var;

    /* Map to 7x7 P (reuse storage but we actually operate on 6x6 subset) */
    /* Store the 6x6 error covariance in the first 36 elements of the 49-element P array */
    for (int i = 0; i < 3; i++) {
        ahrs->P[i * 7 + i] = att_var;          /* attitude diagonal */
        ahrs->P[(i+3) * 7 + (i+3)] = bias_var; /* bias diagonal */
    }

    ahrs->mag_ref_valid = false;
}

/* ── EKF Prediction ────────────────────────────────────────── */

static void ekf_predict(ahrs_t *ahrs, const float gyro[3], float dt)
{
    /* Bias-corrected gyro */
    float w[3];
    w[0] = gyro[0] - ahrs->gyro_bias[0];
    w[1] = gyro[1] - ahrs->gyro_bias[1];
    w[2] = gyro[2] - ahrs->gyro_bias[2];

    /* Quaternion prediction: q_new = q + 0.5 * Omega(w) * q * dt */
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;

    float dq0 = 0.5f * (-q1*w[0] - q2*w[1] - q3*w[2]) * dt;
    float dq1 = 0.5f * ( q0*w[0] + q2*w[2] - q3*w[1]) * dt;
    float dq2 = 0.5f * ( q0*w[1] - q1*w[2] + q3*w[0]) * dt;
    float dq3 = 0.5f * ( q0*w[2] + q1*w[1] - q2*w[0]) * dt;

    ahrs->q0 = q0 + dq0;
    ahrs->q1 = q1 + dq1;
    ahrs->q2 = q2 + dq2;
    ahrs->q3 = q3 + dq3;
    quat_normalize(&ahrs->q0, &ahrs->q1, &ahrs->q2, &ahrs->q3);

    /* Bias prediction: constant (random walk model) */
    /* ahrs->gyro_bias unchanged */

    /* Covariance prediction: P = F*P*F' + Q
     * Using 6x6 error-state: [attitude_error(3), bias_error(3)]
     *
     * F = | I - [w]x*dt  -I*dt |
     *     | 0             I     |
     *
     * Q = diag(gyro_noise^2*dt^2 (x3), gyro_bias_noise^2*dt (x3))
     */

    /* Extract 6x6 P from the 7x7 storage (rows/cols 0-5, stride 7) */
    float P[36], Pnew[36];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P[P6(i,j)] = ahrs->P[i * 7 + j];
        }
    }

    /* Build F matrix */
    float F[36];
    mat6_identity(F);

    /* F[0:3, 0:3] = I - [w]x * dt */
    float wS[9];
    skew(w, wS);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[P6(i,j)] -= wS[i*3+j] * dt;
        }
    }

    /* F[0:3, 3:6] = -I * dt */
    for (int i = 0; i < 3; i++) {
        F[P6(i, i+3)] = -dt;
    }

    /* P = F * P * F' + Q */
    float FP[36], FT[36];
    mat6_mul(F, P, FP);
    mat6_transpose(F, FT);
    mat6_mul(FP, FT, Pnew);

    /* Add Q */
    float gn2dt2 = ahrs->gyro_noise * ahrs->gyro_noise * dt * dt;
    float bn2dt  = ahrs->gyro_bias_noise * ahrs->gyro_bias_noise * dt;
    for (int i = 0; i < 3; i++) {
        Pnew[P6(i,i)] += gn2dt2;
        Pnew[P6(i+3,i+3)] += bn2dt;
    }

    /* Store back to 7x7 */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ahrs->P[i * 7 + j] = Pnew[P6(i,j)];
        }
    }
}

/* ── EKF Accelerometer Update ──────────────────────────────── */

static void ekf_update_accel(ahrs_t *ahrs, const float accel[3])
{
    float ax = accel[0], ay = accel[1], az = accel[2];
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;

    /* Reject if acceleration is far from 1g (dynamic motion) */
    float g_ratio = norm / 9.81f;
    if (g_ratio < 0.8f || g_ratio > 1.2f) return;

    float inv = 1.0f / norm;
    ax *= inv; ay *= inv; az *= inv;

    /* Expected gravity in body frame: g_body = R' * [0,0,1] (NED convention, gravity = +Z down)
     * For NED: gravity vector in NED = [0, 0, 9.81], normalized = [0, 0, 1]
     * g_body = R^T * [0, 0, 1] = third column of R^T = third row of R */
    float R[9];
    quat_to_dcm(ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3, R);

    /* Predicted gravity in body frame (third row of R, for NED gravity down) */
    float gx_pred = R[6];  /* R[2][0] */
    float gy_pred = R[7];  /* R[2][1] */
    float gz_pred = R[8];  /* R[2][2] */

    /* Innovation: measured - predicted */
    float y[3];
    y[0] = ax - gx_pred;
    y[1] = ay - gy_pred;
    y[2] = az - gz_pred;

    /* Measurement Jacobian H (3x6): H = [dh/d_theta | 0(3x3)]
     * dh/d_theta = -[g_pred]x  (skew symmetric of predicted gravity) */
    float H[3 * ERR_DIM];
    memset(H, 0, sizeof(H));

    /* -skew(g_pred) */
    H[0*ERR_DIM + 0] =  0.0f;
    H[0*ERR_DIM + 1] = -gz_pred;
    H[0*ERR_DIM + 2] =  gy_pred;
    H[1*ERR_DIM + 0] =  gz_pred;
    H[1*ERR_DIM + 1] =  0.0f;
    H[1*ERR_DIM + 2] = -gx_pred;
    H[2*ERR_DIM + 0] = -gy_pred;
    H[2*ERR_DIM + 1] =  gx_pred;
    H[2*ERR_DIM + 2] =  0.0f;

    /* S = H * P * H' + R_meas */
    float P[36];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P[P6(i,j)] = ahrs->P[i * 7 + j];

    /* HP = H * P (3x6) */
    float HP[3 * ERR_DIM];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < ERR_DIM; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++) {
                sum += H[i*ERR_DIM + k] * P[P6(k,j)];
            }
            HP[i*ERR_DIM + j] = sum;
        }
    }

    /* S = HP * H' + R (3x3) */
    float S[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++) {
                sum += HP[i*ERR_DIM + k] * H[j*ERR_DIM + k]; /* H' */
            }
            S[i*3+j] = sum;
        }
        S[i*3+i] += ahrs->accel_noise * ahrs->accel_noise;
    }

    /* Invert S (3x3) using Cramer's rule */
    float det = S[0]*(S[4]*S[8]-S[5]*S[7])
              - S[1]*(S[3]*S[8]-S[5]*S[6])
              + S[2]*(S[3]*S[7]-S[4]*S[6]);
    if (fabsf(det) < 1e-10f) return;
    float inv_det = 1.0f / det;

    float Si[9]; /* S inverse */
    Si[0] = (S[4]*S[8]-S[5]*S[7]) * inv_det;
    Si[1] = (S[2]*S[7]-S[1]*S[8]) * inv_det;
    Si[2] = (S[1]*S[5]-S[2]*S[4]) * inv_det;
    Si[3] = (S[5]*S[6]-S[3]*S[8]) * inv_det;
    Si[4] = (S[0]*S[8]-S[2]*S[6]) * inv_det;
    Si[5] = (S[2]*S[3]-S[0]*S[5]) * inv_det;
    Si[6] = (S[3]*S[7]-S[4]*S[6]) * inv_det;
    Si[7] = (S[1]*S[6]-S[0]*S[7]) * inv_det;
    Si[8] = (S[0]*S[4]-S[1]*S[3]) * inv_det;

    /* K = P * H' * S_inv (6x3) */
    /* First: PH' (6x3) */
    float PHt[ERR_DIM * 3];
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++) {
                sum += P[P6(i,k)] * H[j*ERR_DIM + k]; /* H' */
            }
            PHt[i*3 + j] = sum;
        }
    }

    /* K = PH' * Si (6x3) */
    float K[ERR_DIM * 3];
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += PHt[i*3+k] * Si[k*3+j];
            }
            K[i*3+j] = sum;
        }
    }

    /* State update: dx = K * y */
    float dx[ERR_DIM];
    for (int i = 0; i < ERR_DIM; i++) {
        dx[i] = K[i*3+0]*y[0] + K[i*3+1]*y[1] + K[i*3+2]*y[2];
    }

    /* Apply attitude correction via small-angle quaternion */
    float dtheta[3] = {dx[0], dx[1], dx[2]};
    float dq[4], q_new[4];
    small_angle_quat(dtheta, dq);

    float q_cur[4] = {ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3};
    quat_mul(q_cur, dq, q_new);
    ahrs->q0 = q_new[0]; ahrs->q1 = q_new[1];
    ahrs->q2 = q_new[2]; ahrs->q3 = q_new[3];
    quat_normalize(&ahrs->q0, &ahrs->q1, &ahrs->q2, &ahrs->q3);

    /* Apply bias correction */
    ahrs->gyro_bias[0] += dx[3];
    ahrs->gyro_bias[1] += dx[4];
    ahrs->gyro_bias[2] += dx[5];

    /* Covariance update: P = (I - K*H) * P */
    float KH[36];
    mat6_zero(KH);
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < ERR_DIM; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += K[i*3+k] * H[k*ERR_DIM + j];
            }
            KH[P6(i,j)] = sum;
        }
    }

    float Pnew[36];
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < ERR_DIM; j++) {
            float ij = (i == j) ? 1.0f : 0.0f;
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++) {
                sum += (((i==k) ? 1.0f : 0.0f) - KH[P6(i,k)]) * P[P6(k,j)];
            }
            Pnew[P6(i,j)] = sum;
            (void)ij;
        }
    }

    /* Store back */
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            ahrs->P[i * 7 + j] = Pnew[P6(i,j)];
}

/* ── EKF Magnetometer Update ──────────────────────────────── */

static void ekf_update_mag(ahrs_t *ahrs, const float mag[3])
{
    float mx = mag[0], my = mag[1], mz = mag[2];
    float norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm < 1e-6f) return;
    float inv = 1.0f / norm;
    mx *= inv; my *= inv; mz *= inv;

    /* Rotate mag to NED frame to get reference */
    float R[9];
    quat_to_dcm(ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3, R);

    /* m_ned = R * m_body */
    float mn_x = R[0]*mx + R[1]*my + R[2]*mz;
    float mn_y = R[3]*mx + R[4]*my + R[5]*mz;
    float mn_z = R[6]*mx + R[7]*my + R[8]*mz;

    /* Update magnetic reference (horizontal + vertical components) */
    if (!ahrs->mag_ref_valid) {
        ahrs->mag_ref_x = sqrtf(mn_x*mn_x + mn_y*mn_y);
        ahrs->mag_ref_z = mn_z;
        ahrs->mag_ref_valid = true;
    }

    /* Expected mag in body frame: m_pred = R' * [ref_x, 0, ref_z] */
    float ref[3] = {ahrs->mag_ref_x, 0.0f, ahrs->mag_ref_z};
    float mx_pred = R[0]*ref[0] + R[3]*ref[1] + R[6]*ref[2]; /* R' * ref */
    float my_pred = R[1]*ref[0] + R[4]*ref[1] + R[7]*ref[2];
    float mz_pred = R[2]*ref[0] + R[5]*ref[1] + R[8]*ref[2];

    /* Innovation */
    float y[3];
    y[0] = mx - mx_pred;
    y[1] = my - my_pred;
    y[2] = mz - mz_pred;

    /* Measurement Jacobian: H = [-skew(m_pred) | 0(3x3)] */
    float H[3 * ERR_DIM];
    memset(H, 0, sizeof(H));
    H[0*ERR_DIM + 0] =  0.0f;
    H[0*ERR_DIM + 1] = -mz_pred;
    H[0*ERR_DIM + 2] =  my_pred;
    H[1*ERR_DIM + 0] =  mz_pred;
    H[1*ERR_DIM + 1] =  0.0f;
    H[1*ERR_DIM + 2] = -mx_pred;
    H[2*ERR_DIM + 0] = -my_pred;
    H[2*ERR_DIM + 1] =  mx_pred;
    H[2*ERR_DIM + 2] =  0.0f;

    /* Same Kalman update math as accel */
    float P[36];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P[P6(i,j)] = ahrs->P[i * 7 + j];

    float HP[3 * ERR_DIM];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < ERR_DIM; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++)
                sum += H[i*ERR_DIM + k] * P[P6(k,j)];
            HP[i*ERR_DIM + j] = sum;
        }
    }

    float S[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++)
                sum += HP[i*ERR_DIM + k] * H[j*ERR_DIM + k];
            S[i*3+j] = sum;
        }
        S[i*3+i] += ahrs->mag_noise * ahrs->mag_noise;
    }

    float det = S[0]*(S[4]*S[8]-S[5]*S[7])
              - S[1]*(S[3]*S[8]-S[5]*S[6])
              + S[2]*(S[3]*S[7]-S[4]*S[6]);
    if (fabsf(det) < 1e-10f) return;
    float inv_det = 1.0f / det;

    float Si[9];
    Si[0] = (S[4]*S[8]-S[5]*S[7]) * inv_det;
    Si[1] = (S[2]*S[7]-S[1]*S[8]) * inv_det;
    Si[2] = (S[1]*S[5]-S[2]*S[4]) * inv_det;
    Si[3] = (S[5]*S[6]-S[3]*S[8]) * inv_det;
    Si[4] = (S[0]*S[8]-S[2]*S[6]) * inv_det;
    Si[5] = (S[2]*S[3]-S[0]*S[5]) * inv_det;
    Si[6] = (S[3]*S[7]-S[4]*S[6]) * inv_det;
    Si[7] = (S[1]*S[6]-S[0]*S[7]) * inv_det;
    Si[8] = (S[0]*S[4]-S[1]*S[3]) * inv_det;

    float PHt[ERR_DIM * 3];
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++)
                sum += P[P6(i,k)] * H[j*ERR_DIM + k];
            PHt[i*3 + j] = sum;
        }
    }

    float K[ERR_DIM * 3];
    for (int i = 0; i < ERR_DIM; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
                sum += PHt[i*3+k] * Si[k*3+j];
            K[i*3+j] = sum;
        }
    }

    float dx[ERR_DIM];
    for (int i = 0; i < ERR_DIM; i++)
        dx[i] = K[i*3+0]*y[0] + K[i*3+1]*y[1] + K[i*3+2]*y[2];

    /* Apply correction */
    float dtheta[3] = {dx[0], dx[1], dx[2]};
    float dq[4], q_new[4];
    small_angle_quat(dtheta, dq);
    float q_cur[4] = {ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3};
    quat_mul(q_cur, dq, q_new);
    ahrs->q0 = q_new[0]; ahrs->q1 = q_new[1];
    ahrs->q2 = q_new[2]; ahrs->q3 = q_new[3];
    quat_normalize(&ahrs->q0, &ahrs->q1, &ahrs->q2, &ahrs->q3);

    ahrs->gyro_bias[0] += dx[3];
    ahrs->gyro_bias[1] += dx[4];
    ahrs->gyro_bias[2] += dx[5];

    /* P = (I - K*H) * P */
    float KH[36];
    mat6_zero(KH);
    for (int i = 0; i < ERR_DIM; i++)
        for (int j = 0; j < ERR_DIM; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
                sum += K[i*3+k] * H[k*ERR_DIM + j];
            KH[P6(i,j)] = sum;
        }

    float Pnew[36];
    for (int i = 0; i < ERR_DIM; i++)
        for (int j = 0; j < ERR_DIM; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ERR_DIM; k++)
                sum += (((i==k) ? 1.0f : 0.0f) - KH[P6(i,k)]) * P[P6(k,j)];
            Pnew[P6(i,j)] = sum;
        }

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            ahrs->P[i * 7 + j] = Pnew[P6(i,j)];
}

/* ── Public Update Functions ───────────────────────────────── */

void ahrs_update(ahrs_t *ahrs,
                 const float gyro[3],
                 const float accel[3],
                 const float mag[3],
                 float dt)
{
    if (dt <= 0.0f || dt > 0.1f) return;

    /* EKF prediction step */
    ekf_predict(ahrs, gyro, dt);

    /* Accelerometer measurement update (roll/pitch) */
    ekf_update_accel(ahrs, accel);

    /* Magnetometer measurement update (yaw) */
    ekf_update_mag(ahrs, mag);
}

void ahrs_update_imu(ahrs_t *ahrs,
                     const float gyro[3],
                     const float accel[3],
                     float dt)
{
    if (dt <= 0.0f || dt > 0.1f) return;

    ekf_predict(ahrs, gyro, dt);
    ekf_update_accel(ahrs, accel);
}

void ahrs_get_quaternion(const ahrs_t *ahrs, float q[4])
{
    q[0] = ahrs->q0;
    q[1] = ahrs->q1;
    q[2] = ahrs->q2;
    q[3] = ahrs->q3;
}

void ahrs_get_euler(const ahrs_t *ahrs, float *roll, float *pitch, float *yaw)
{
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;

    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf((float)(M_PI / 2.0), sinp);
    } else {
        *pitch = asinf(sinp);
    }

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void ahrs_get_gyro_bias(const ahrs_t *ahrs, float bias[3])
{
    bias[0] = ahrs->gyro_bias[0];
    bias[1] = ahrs->gyro_bias[1];
    bias[2] = ahrs->gyro_bias[2];
}
