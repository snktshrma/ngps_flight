#include "ap_ukf/node/fusionCore.h"
#include <iostream>

Fusion::Fusion(double alpha, double beta, double kappa)
    : filter_(alpha, beta, kappa,
              std::bind(&Fusion::stateTransitionFunction, this,
                        std::placeholders::_1, std::placeholders::_2)),
      isInitialized_(false),
      lastMeasurementTime_(0.0) {
  setupProcessModel();
  setupMeasurementModels();

  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;
  Q.setZero();
  Q.diagonal() << 0.01, 0.01, 0.1, 0.1;
  filter_.setProcessNoise(Q);
  initialP_.setIdentity();
  initialP_ *= 1e-9;

  reset();
}

void Fusion::reset() {
  filter_.setCovariance(initialP_);
  filter_.setState(Eigen::Matrix<double, STATE_SIZE, 1>::Zero());

  lastMeasurementTime_ = 0.0;
  isInitialized_ = false;
}

void Fusion::predict(double deltaT) { filter_.predict(deltaT); }

void Fusion::processMeasurement(Measurement measurement) {
  double deltaT = 0.0;

  if (measurement.type == MeasurementTypeBootstrap) {
    Eigen::Matrix<double, STATE_SIZE, 1> x0;
    for (int i = 0; i < STATE_SIZE; ++i) {
      x0(i) = measurement.measurement(i);
    }
    filter_.setState(x0);
    if (measurement.covariance.rows() == STATE_SIZE &&
        measurement.covariance.cols() == STATE_SIZE) {
      Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P0;
      for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
          P0(i, j) = measurement.covariance(i, j);
        }
      }
      filter_.setCovariance(P0);
    } else {
      filter_.setCovariance(initialP_);
    }
    isInitialized_ = true;
    lastMeasurementTime_ = measurement.time;
    return;
  }

  if (!isInitialized_) {
    return;
  }

  deltaT = measurement.time - lastMeasurementTime_;

  if (deltaT > 100000.0) {
    deltaT = 0.01;
  } else if (deltaT < 0.0) {
    return;
  }

  filter_.predict(deltaT);

  if (measurement.type == MeasurementTypeVio) {
    Eigen::Matrix<double, VIO_SIZE, 1> z = measurement.measurement;
    Eigen::Matrix<double, VIO_SIZE, VIO_SIZE> R = measurement.covariance;
    filter_.update(z, R, vioModel_);
  } else if (measurement.type == MeasurementTypeVps) {
    Eigen::Matrix<double, VPS_SIZE, 1> z = measurement.measurement;
    Eigen::Matrix<double, VPS_SIZE, VPS_SIZE> R = measurement.covariance;
    filter_.update(z, R, vpsModel_);
  }

  if (deltaT >= 0.0) {
    lastMeasurementTime_ = measurement.time;
  }
}

Eigen::Matrix<double, STATE_SIZE, 1> Fusion::stateTransitionFunction(
    const Eigen::Matrix<double, STATE_SIZE, 1> &x, double deltaT) {
  Eigen::Matrix<double, STATE_SIZE, 1> newState;
  newState(StateX) = x(StateX) + x(StateVx) * deltaT;
  newState(StateY) = x(StateY) + x(StateVy) * deltaT;
  newState(StateVx) = x(StateVx);
  newState(StateVy) = x(StateVy);
  return newState;
}

void Fusion::setupProcessModel() {
  filter_.setDifferenceFunction(
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &a,
         const Eigen::Matrix<double, STATE_SIZE, 1> &b)
          -> Eigen::Matrix<double, STATE_SIZE, 1> { return a - b; });

  filter_.setMeanFunction(
      [](const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE * 2 + 1> &sigmaPoints,
         const Eigen::Matrix<double, STATE_SIZE * 2 + 1, 1> &weights)
          -> Eigen::Matrix<double, STATE_SIZE, 1> {
        Eigen::Matrix<double, STATE_SIZE, 1> mean;
        mean.setZero();
        for (int i = 0; i < sigmaPoints.cols(); i++) {
          mean += sigmaPoints.col(i) * weights(i);
        }
        return mean;
      });
}

void Fusion::setupMeasurementModels() {
  vioModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, VIO_SIZE, 1> {
    Eigen::Matrix<double, VIO_SIZE, 1> measurement;
    measurement(VioX) = state(StateX);
    measurement(VioY) = state(StateY);
    measurement(VioVx) = state(StateVx);
    measurement(VioVy) = state(StateVy);
    return measurement;
  };

  vpsModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, VPS_SIZE, 1> {
    Eigen::Matrix<double, VPS_SIZE, 1> measurement;
    measurement(0) = state(StateX);
    measurement(1) = state(StateY);
    return measurement;
  };
}

void Fusion::setInitialCovariance(
    const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &P) {
  initialP_ = P;
  reset();
}

void Fusion::setProcessNoise(
    const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &Q) {
  filter_.setProcessNoise(Q);
  reset();
}

Eigen::Matrix<double, STATE_SIZE, 1> &Fusion::getState() {
  return filter_.getState();
}

Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &Fusion::getCovariance() {
  return filter_.getCovariance();
}

void Fusion::setState(const Eigen::Matrix<double, STATE_SIZE, 1> &x) {
  filter_.setState(x);
}

bool Fusion::isInitialized() { return isInitialized_; }

double Fusion::getLastMeasurementTime() { return lastMeasurementTime_; }
