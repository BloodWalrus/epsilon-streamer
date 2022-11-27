use glam::{vec4, Quat, Vec4};

// https://github.com/arduino-libraries/MadgwickAHRS
pub struct Ahrs {
    beta: f32,
    q: Quat,
    inv_sample_freq: f32,
}

impl Ahrs {
    pub fn new(freq: f32) -> Self {
        Self {
            beta: 0.1,
            q: Quat::IDENTITY,
            inv_sample_freq: freq.recip(),
        }
    }

    pub fn rotation(&self) -> Quat {
        self.q
    }

    pub fn update_imu(&mut self, mut gyro: Vec3A, mut accel: Vec3A) {
        gyro *= 0.0174533;
        let mut q_dot1 = 0.5 * (-self.q.x * gx - self.q.y * gy - self.q.z * gz);
        let mut q_dot2 = 0.5 * (self.q.w * gx + self.q.y * gz - self.q.z * gy);
        let mut q_dot3 = 0.5 * (self.q.w * gy - self.q.x * gz + self.q.z * gx);
        let mut q_dot4 = 0.5 * (self.q.w * gz + self.q.x * gy - self.q.y * gx);

        if accel != Vec3A::ZERO {
            // Normalise accelerometer measurement
            accel = accel.normalize();

            // Auxiliary variables to avoid repeated arithmetic
            let _2q0 = 2.0 * self.q.w;
            let _2q1 = 2.0 * self.q.x;
            let _2q2 = 2.0 * self.q.y;
            let _2q3 = 2.0 * self.q.z;
            let _4q0 = 4.0 * self.q.w;
            let _4q1 = 4.0 * self.q.x;
            let _4q2 = 4.0 * self.q.y;
            let _8q1 = 8.0 * self.q.x;
            let _8q2 = 8.0 * self.q.y;
            let q0q0 = self.q.w * self.q.w;
            let q1q1 = self.q.x * self.q.x;
            let q2q2 = self.q.y * self.q.y;
            let q3q3 = self.q.z * self.q.z;

            // Gradient decent algorithm corrective step
            let mut s = vec4(
                _4q0 * q2q2 + _2q2 * accel.x + _4q0 * q1q1 - _2q1 * accel.y,
                _4q1 * q3q3 - _2q3 * accel.x + 4.0 * q0q0 * self.q.x - _2q0 * accel.y - _4q1
                    + _8q1 * q1q1
                    + _8q1 * q2q2
                    + _4q1 * accel.z,
                4.0 * q0q0 * self.q.y + _2q0 * accel.x + _4q2 * q3q3 - _2q3 * accel.y - _4q2
                    + _8q2 * q1q1
                    + _8q2 * q2q2
                    + _4q2 * accel.z,
                4.0 * q1q1 * self.q.z - _2q1 * accel.x + 4.0 * q2q2 * self.q.z - _2q2 * accel.y,
            );
            s = s.normalize();

            // Apply feedback step
            q_dot1 -= self.beta * s.x;
            q_dot2 -= self.beta * s.y;
            q_dot3 -= self.beta * s.z;
            q_dot4 -= self.beta * s.w;
        }

        // Integrate rate of change of quaternion to yield quaternion
        self.q.w += q_dot1 * self.inv_sample_freq;
        self.q.x += q_dot2 * self.inv_sample_freq;
        self.q.y += q_dot3 * self.inv_sample_freq;
        self.q.z += q_dot4 * self.inv_sample_freq;

        // Normalise quaternion
        self.q = self.q.normalize();
    }
}
