#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>

// 4D horizontal state: local meters (x, y, vx, vy). See SYSTEM_PLAN.md.
enum State {
  StateX = 0,
  StateY,
  StateVx,
  StateVy
};
const int STATE_SIZE = 4;

// VIO observes position + velocity in the local frame (4D)
const int VIO_SIZE = 4;

const int VPS_SIZE = 2;

enum Vio {
  VioX = 0,
  VioY,
  VioVx,
  VioVy
};

enum MeasurementType {
  MeasurementTypeNone = 0,
  MeasurementTypeVio,
  MeasurementTypeVps,
  MeasurementTypeBootstrap
};

struct Measurement {
  MeasurementType type;
  Eigen::VectorXd measurement;
  Eigen::MatrixXd covariance;
  double time;
  double mahalanobisThreshold;

  Measurement()
      : type(MeasurementTypeNone),
        time(0.0),
        mahalanobisThreshold(std::numeric_limits<double>::max()) {}

  Measurement(size_t size) : measurement(size), covariance(size, size) {
    type = MeasurementTypeNone;
    time = 0.0;
    mahalanobisThreshold = std::numeric_limits<double>::max();
  }

  bool operator()(const std::shared_ptr<Measurement>& a,
                  const std::shared_ptr<Measurement>& b) {
    return a->time > b->time;
  }
};
typedef std::shared_ptr<Measurement> MeasurementPtr;
