#pragma once

#include <eigen3/Eigen/Dense>

template <typename T>
T clip(const T &n, const T &lower, const T &upper) {
  return std::max(lower, std::min(n, upper));
}

template <typename Type>
Eigen::Quaternion<Type> ClipQuaternion(const Eigen::Quaternion<Type> &x,
                                       const Type lower, const Type upper) {
  Eigen::Quaternion<Type> q;
  if (upper > lower) {
    q.w() = clip(x.w(), lower, upper);
    q.x() = clip(x.x(), lower, upper);
    q.y() = clip(x.y(), lower, upper);
    q.z() = clip(x.z(), lower, upper);
  } else {
    q.w() = clip(x.w(), upper, lower);
    q.x() = clip(x.x(), upper, lower);
    q.y() = clip(x.y(), upper, lower);
    q.z() = clip(x.z(), upper, lower);
  }
  return q;
}

template <typename Type, int M, int N>
Eigen::Matrix<Type, M, N> ClipMatrix(const Eigen::Matrix<Type, M, N> &x,
                                     const Type lower, const Type upper) {
  Eigen::Matrix<Type, M, N> matrix;
  if (upper > lower) {
    for (int i = 0; i < M; ++i) {
      for (int j = 0; j < N; ++j) {
        matrix(i, j) = clip(x(i, j), lower, upper);
      }
    }
  } else {
    for (int i = 0; i < M; ++i) {
      for (int j = 0; j < N; ++j) {
        matrix(i, j) = clip(x(i, j), upper, lower);
      }
    }
  }
  return matrix;
}

template <typename Type, int M, int N>
Eigen::Matrix<Type, M, N> ClipMatrix(const Eigen::Matrix<Type, M, N> &x,
                                     const Eigen::Matrix<Type, M, N> &lower,
                                     const Eigen::Matrix<Type, M, N> &upper) {
  Eigen::Matrix<Type, M, N> m;
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      if (lower(i, j) < upper(i, j)) {
        m(i, j) = clip(x(i, j), lower(i, j), upper(i, j));
      } else {
        m(i, j) = clip(x(i, j), upper(i, j), lower(i, j));
      }
    }
  }
  return m;
}

inline double square(double a) { return a * a; }

template <typename Floating>
Floating wrap_floating(Floating x, Floating low, Floating high) {
  if (low <= x && x < high) {
    return x;
  }
  const auto range = high - low;
  const auto inv_range = Floating(1) / range;
  const auto num_wraps = floor((x - low) * inv_range);
  return x - range * num_wraps;
}

inline double wrap(double x, double low, double high) {
  return wrap_floating(x, low, high);
}

inline float wrap(float x, float low, float high) {
  return wrap_floating(x, low, high);
}

template <typename Type>
Type wrap_pi(Type x) {
  return wrap(x, Type(-M_PI), Type(M_PI));
}

inline bool ShouldUse321RotationSequence(const Eigen::Matrix3d &R) {
  return abs(R(2, 0)) < abs(R(2, 1));
}

inline double Euler321Yaw(const Eigen::Matrix3d &R) {
  return atan2(R(1, 0), R(0, 0));
}

inline double Euler321Yaw(const Eigen::Quaterniond &q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
}

inline double Euler312Yaw(const Eigen::Matrix3d &R) {
  return atan2(-R(0, 1), R(1, 1));
}

inline double Euler312Yaw(const Eigen::Quaterniond &q) {
  return atan2(2.0 * (q.w() * q.z() - q.x() * q.y()),
               q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z());
}

Eigen::Matrix3d UpdateYawInRotationMatrix(double yaw, const Eigen::Matrix3d &R);
double kahan_summation(double sum, double input, double &accumulator);
Eigen::Matrix3d TaitBryan312ToRotationMatrix(const Eigen::Vector3d &euler);
Eigen::Matrix3d UpdateYawInRotationMatrix(double yaw, const Eigen::Matrix3d &R);
