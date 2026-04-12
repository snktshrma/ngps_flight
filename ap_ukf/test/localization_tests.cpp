#include <ap_ukf/node/fusionCore.h>
#include <gtest/gtest.h>

TEST(FusionTests4d, constantVelocityPredict) {
  Fusion fusion(1e-2, 2, 0);
  fusion.reset();

  Measurement boot(STATE_SIZE);
  boot.type = MeasurementTypeBootstrap;
  boot.time = 0.0;
  boot.measurement.resize(STATE_SIZE);
  boot.measurement << 0.0, 0.0, 1.0, 0.5;
  boot.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1;
  fusion.processMeasurement(boot);
  EXPECT_TRUE(fusion.isInitialized());

  fusion.predict(0.1);
  EXPECT_NEAR(fusion.getState()(StateX), 0.1, 1e-6);
  EXPECT_NEAR(fusion.getState()(StateY), 0.05, 1e-6);
  EXPECT_NEAR(fusion.getState()(StateVx), 1.0, 1e-6);
  EXPECT_NEAR(fusion.getState()(StateVy), 0.5, 1e-6);
}

TEST(FusionTests4d, vioUpdate) {
  Fusion fusion(1e-2, 2, 0);
  Measurement boot(STATE_SIZE);
  boot.type = MeasurementTypeBootstrap;
  boot.time = 0.0;
  boot.measurement.resize(STATE_SIZE);
  boot.measurement << 0.0, 0.0, 0.0, 0.0;
  boot.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  fusion.processMeasurement(boot);

  Measurement vio(VIO_SIZE);
  vio.type = MeasurementTypeVio;
  vio.time = 0.05;
  vio.measurement.resize(VIO_SIZE);
  vio.measurement << 0.1, 0.2, 0.3, 0.4;
  vio.covariance = Eigen::MatrixXd::Identity(VIO_SIZE, VIO_SIZE) * 0.5;
  fusion.processMeasurement(vio);

  EXPECT_NEAR(fusion.getState()(StateX), 0.1, 0.2);
  EXPECT_NEAR(fusion.getState()(StateY), 0.2, 0.2);
}
