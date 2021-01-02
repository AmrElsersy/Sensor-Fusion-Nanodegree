#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */

UKF::UKF() {

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);  
  Xsig_pred_.fill(0.0);

  // last step time in us
  time_us_ = 0.0;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    this->initialize(meas_package);
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }

  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  this->Prediction(delta_t);

  // Update
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    this->UpdateLidar(meas_package);
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    this->UpdateRadar(meas_package);

}

void UKF::initialize(MeasurementPackage meas_package)
{
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
      x_ << 
      meas_package.raw_measurements_(0), 
      meas_package.raw_measurements_(1), 
      0, 0, 
      0.0;
      
      // P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
      //       0, std_laspy_*std_laspy_, 0, 0, 0,
      //       0, 0, 1, 0, 0,
      //       0, 0, 0, 1, 0,
      //       0, 0, 0, 0, 1;
            P_ << 1, 0, 0, 0, 0,
                  0, 1, 0, 0, 0,
                  0, 0, 1, 0, 0,
                  0, 0, 0, 1, 0,
                  0, 0, 0, 0, 1;

  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rhodot = meas_package.raw_measurements_(2);
      double x = rho * cos(phi);
      double y = rho * sin(phi);
      double vx = rhodot * cos(phi);
      double vy = rhodot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);
      x_ << x, y, 0, 0, 0;
      
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
  }

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */


  // =============== Augumentation ==========================
  lambda_ = 3 - n_x_;

  // Sigma points with new dimentions for augumentation
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);

  // mean and covarience of Augumented 
  VectorXd x_aug_ = VectorXd(n_aug_);
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);  

  // new mean 
  x_aug_.fill(0);
  x_aug_.head(5) = x_;

  // Error Covarience
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0,
        0, std_yawdd_*std_yawdd_;

  // Augment the Error Covarience Matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_.bottomRightCorner(2, 2) = Q;


  MatrixXd A_aug_ = P_aug_.llt().matrixL();
  
  Xsig_aug_.col(0) = x_aug_;
  for(int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i+1) = x_aug_ + std::sqrt(lambda_+n_aug_) * A_aug_.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - std::sqrt(lambda_+n_aug_)* A_aug_.col(i);
  }

  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + (v/yawd) * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + (v/yawd) * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // Get the mean & Covarience of the predicted sigma points
  VectorXd Xsig_mean = VectorXd(n_x_);
  Xsig_mean.fill(0.0);
  for (int i =0; i < 2*n_aug_+1; i++)
    Xsig_mean +=  weights_(i) * Xsig_pred_.col(i);

  // predicted state covariance matrix
  MatrixXd P(n_x_,n_x_);
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - Xsig_mean;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  // Update State and Covarience
  this->x_ = Xsig_mean;
  this->P_ = P;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // convert the XSig_pred to ZSig
  MatrixXd ZSig = MatrixXd(2, 2*n_aug_+1);
  ZSig.fill(0.0);

  for (int i =0 ; i < 2*n_aug_+1; i++)
  {
    auto x_sig = Xsig_pred_.col(i);
    ZSig.col(i) << x_sig(0), 
                   x_sig(1);
  }

  // Z Sig Mean
  VectorXd z_mean = VectorXd(2);
  z_mean.fill(0.0);
  for (int i =0 ; i < 2*n_aug_+1; i++)
    z_mean += weights_(i) * ZSig.col(i);

  // S Covarience Matrix
  MatrixXd S_covarience = MatrixXd(2, 2);
  S_covarience.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = ZSig.col(i) - z_mean;
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < - M_PI) z_diff(1) += 2. * M_PI;    

    S_covarience += weights_(i) * z_diff * z_diff.transpose();
  }

  // Add Error Matrix R
  MatrixXd R = MatrixXd(2,2);
  R << std_laspx_*std_laspx_, 0, 0, std_laspy_*std_laspy_;
  S_covarience += R;

  // Z Measurments
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);  

  // Tc Matrix
  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);
  
  // used for KG
  for (int i = 0; i < 2*n_aug_+ 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;    

    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    
    VectorXd z_diff = ZSig.col(i) - z_mean;
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < - M_PI) z_diff(1) += 2. * M_PI;    

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  
  // Kalman Gain
  MatrixXd KG = Tc * S_covarience.inverse();

  VectorXd z_diff = z - z_mean;
  while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < - M_PI) z_diff(1) += 2. * M_PI;    

  // Update
  x_ = x_ + KG * z_diff;
  P_ = P_ - KG * S_covarience * KG.transpose();

  NIS_laser_ = z_diff.transpose() * S_covarience.inverse() * z_diff;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // convert the XSig_pred to ZSig
  MatrixXd ZSig = MatrixXd(3, 2*n_aug_+1);
  ZSig.fill(0.0);

  for (int i =0 ; i < 2*n_aug_+1; i++)
  {
    auto x_sig = Xsig_pred_.col(i);

    double px = x_sig(0);
    double py = x_sig(1);
    double v = x_sig(2);
    double yaw = x_sig(3);
    double yaw_d = x_sig(4);
    
    auto rho = sqrt(px*px+py*py);
    auto phi = atan2(py,px);
    auto rho_d = (px*cos(yaw)*v+py*sin(yaw)*v) / rho;
    
    ZSig.col(i) << rho,
                   phi,
                   rho_d;
  }

  // Z Sig Mean
  VectorXd z_mean = VectorXd(3);
  z_mean.fill(0.0);
  for (int i =0 ; i < 2*n_aug_+1; i++)
    z_mean += weights_(i) * ZSig.col(i);

  // S Covarience Matrix
  MatrixXd S_covarience = MatrixXd(3, 3);
  S_covarience.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = ZSig.col(i) - z_mean;
    while (z_diff(1) > M_PI)  z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < - M_PI) z_diff(1) += 2. * M_PI;    
    S_covarience += weights_(i) * z_diff * z_diff.transpose();
  }

  // Add Error Matrix R
  MatrixXd R = MatrixXd(3,3);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;  
  S_covarience += R;

  // Z Measurments
  VectorXd z = VectorXd(3);
  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);  

  //Correlation Matrix Tc
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  
  // used for KG
  for (int i = 0; i < 2*n_aug_+ 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;    

    while (x_diff(3) > M_PI) 
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) 
      x_diff(3) += 2. * M_PI;
    
    VectorXd z_diff = ZSig.col(i) - z_mean;
    while (z_diff(1) > M_PI) 
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) 
      z_diff(1) += 2. * M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  
  // Kalman Gain
  MatrixXd KG = Tc * S_covarience.inverse();

  // Update
  VectorXd z_diff = z - z_mean;

  //normalize angles
  while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

  x_ = x_ + KG * z_diff;
  P_ = P_ - KG * S_covarience * KG.transpose();  

  NIS_radar_ = z_diff.transpose() * S_covarience.inverse() * z_diff;
}

