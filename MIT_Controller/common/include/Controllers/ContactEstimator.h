#ifndef PROJECT_CONTACTESTIMATOR_H
#define PROJECT_CONTACTESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"

/*!
 * A "passthrough" contact estimator which returns the expected contact state
 */
template <typename T>
class ContactEstimator : public GenericEstimator<T> {
 public:

  /*!
   * Set the estimated contact by copying the exptected contact state into the
   * estimated contact state
   */
  virtual void run() {
    this->_stateEstimatorData.result->contactEstimate =
        *this->_stateEstimatorData.contactPhase;
  }

  /*!
   * Set up the contact estimator
   */
  virtual void print_estimate()
  {
    printf("\n \t\tCONTACT ESTIMATE\n");
    pretty_print(this->_stateEstimatorData.result->contactEstimate, std::cout, "Contact Estimate");
  }

  virtual void log_estimate()
  {
    this->_saveFileManager->saveValue("CONTACT ESTIMATE" );
    this->_saveFileManager->saveVector(this->_stateEstimatorData.result->contactEstimate );
  }
  virtual void setup() {}
};

#endif  // PROJECT_CONTACTESTIMATOR_H
