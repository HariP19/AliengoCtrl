#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/LegController.h"
#include "Types/IMUTypes.h"
#include "Utilities/Utilities_print.h"
#include "Utilities/save_file.h"


template <typename T>
struct StateEstimate 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;
  Vec3<T> position;
  Vec3<T> vBody;
  Quat<T> orientation;
  Vec3<T> omegaBody;
  RotMat<T> rBody;
  Vec3<T> rpy;

  Vec3<T> omegaWorld;
  Vec3<T> vWorld;
  Vec3<T> aBody, aWorld;

};

template <typename T>
struct StateEstimatorData 
{
  StateEstimate<T>* result;  // where to write the output to
  VectorNavData* vectorNavData;
  LegControllerData<T>* legControllerData;
  Vec4<T>* contactPhase;
  RobotControlParameters* parameters;
  // CheaterState<double>* cheaterState;
};

template <typename T>
class GenericEstimator 
{
 public:
  virtual void run() = 0;
  virtual void setup() = 0;
  virtual void print_estimate() = 0;
  virtual void log_estimate() = 0;

  void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }
  void setSaveFileManager(SaveFileManager<T> *_sfManager){_saveFileManager = _sfManager; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> _stateEstimatorData;
  SaveFileManager<T> *_saveFileManager;
};

template <typename T>
class StateEstimatorContainer {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
	 * Construct a new state estimator container
	 */
	StateEstimatorContainer(
  						  //CheaterState<double>* cheaterState,
                          VectorNavData* vectorNavData,
                          LegControllerData<T>* legControllerData,
                          StateEstimate<T>* stateEstimate,
                          RobotControlParameters* parameters,
                          SaveFileManager<T>* saveFileManager
                          ) 
	{
    
		_data.vectorNavData = vectorNavData;
		_data.legControllerData = legControllerData;
		_data.result = stateEstimate;
		_phase = Vec4<T>::Zero();
		_data.contactPhase = &_phase;
		//_data.cheaterState = cheaterState;
		_data.parameters = parameters;
		_saveFileManager = saveFileManager;
	}

	// Run all estimator
	void run() 
	{
		for (auto estimator : _estimators) 
		{
	  		estimator->run();
		}
	}

	void print_estimate()
	{
		for (auto estimator : _estimators)
		{
			estimator->print_estimate();
		}
	}

	void log_estimate()
	{
		_saveFileManager->saveValue("*******************State Estimator Output*******************" );
		for (auto estimator : _estimators)
		{
			estimator->log_estimate();
		}
	}

	const StateEstimate<T>& getResult() { return *_data.result; }
	StateEstimate<T> * getResultHandle() { return _data.result; }


	void setContactPhase(Vec4<T>& phase) 
	{ 
    	*_data.contactPhase = phase; 
	}

	template <typename EstimatorToAdd>
	void addEstimator() 
	{
	    auto* estimator = new EstimatorToAdd();
	    estimator->setData(_data);
	    estimator->setSaveFileManager(_saveFileManager);
	    estimator->setup();
	    _estimators.push_back(estimator);
	}

	template <typename EstimatorToRemove>
	void removeEstimator() 
	{
    
    int nRemoved = 0;
    _estimators.erase(
        std::remove_if(_estimators.begin(), _estimators.end(),
                       [&nRemoved](GenericEstimator<T>* e) {
                         if (dynamic_cast<EstimatorToRemove*>(e)) {
                           delete e;
                           nRemoved++;
                           return true;
                         } else {
                           return false;
                         }
                       }),
        _estimators.end());
	}

	void removeAllEstimators() 
	{
    	for (auto estimator : _estimators) 
    	{
      		delete estimator;
    	}
    	_estimators.clear();
    }

    ~StateEstimatorContainer() 
    {
    	for (auto estimator : _estimators) 
    	{
      		delete estimator;
    	}
    }

private:
	StateEstimatorData<T> _data;
	SaveFileManager<T>* _saveFileManager;
	std::vector<GenericEstimator<T>*> _estimators;
	Vec4<T> _phase;

};

#endif  // PROJECT_STATEESTIMATOR_H
