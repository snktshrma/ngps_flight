#pragma once

#include <ap_ukf/core/stateTypes.h>
#include <ap_ukf/core/stateFuser.h>
#include <iostream>
#include <memory>

class Fusion {
 public:
  Fusion(double alpha, double beta, double kappa);

  ~Fusion() = default;

  void reset();

  void processMeasurement(Measurement measurement);

  void predict(double deltaT);

  void setInitialCovariance(
      const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& P);

  void setProcessNoise(const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& Q);

  void setState(const Eigen::Matrix<double, STATE_SIZE, 1>& x);

  Eigen::Matrix<double, STATE_SIZE, 1>& getState();

  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& getCovariance();

  bool isInitialized();

  double getLastMeasurementTime();

 protected:
  Eigen::Matrix<double, STATE_SIZE, 1> stateTransitionFunction(
      const Eigen::Matrix<double, STATE_SIZE, 1>& x, double deltaT);

  void setupProcessModel();

  void setupMeasurementModels();

  Ukf<STATE_SIZE> filter_;

  bool isInitialized_;
  double lastMeasurementTime_;

  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> initialP_;

  MeasurementModel<VIO_SIZE, STATE_SIZE> vioModel_;
  MeasurementModel<VPS_SIZE, STATE_SIZE> vpsModel_;
};
