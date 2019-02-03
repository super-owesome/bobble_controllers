#include <bobble_controllers/Filter.h>

//////////////////////////////////////////////////////////
/// @brief The c'tor constructs the class members.
////////////////////////////////////////////////////////////
Filter::Filter(float numInWeights, float* inWeights,
		       float numOutWeights, float* outWeights) :
         _numInWeights(numInWeights),
         _numOutWeights(numOutWeights)
{
	/// First initialize all weights to 0.0
	for (unsigned int i = 0; i<MAX_FILTER_SIZE; i++){
		_inputWeights[i] = 0.0;
		_outputWeights[i] = 0.0;
	}
	/// Apply the passed in weights.
	for (unsigned int i = 0; i<numInWeights; i++){
		_inputWeights[i] = inWeights[i];
	}
	for (unsigned int i = 0; i<numOutWeights; i++){
		_outputWeights[i] = outWeights[i];
	}
}
//////////////////////////////////////////////////////////
/// @brief The c'tor constructs the class members.
////////////////////////////////////////////////////////////
Filter::Filter() :
         _inputBuffer(),
         _outputBuffer(),
         _numInWeights(0),
         _inputWeights(),
         _numOutWeights(0),
         _outputWeights()
{
	/// initialize all weights to 0.0
	for (unsigned int i = 0; i<MAX_FILTER_SIZE; i++){
		_inputWeights[i] = 0.0;
		_outputWeights[i] = 0.0;
	}
}

////////////////////////////////////////////////////////////
/// @brief Default  d'tor
////////////////////////////////////////////////////////////
Filter::~Filter() {

}
////////////////////////////////////////////////////////////
/// @brief Helper function to initialize the input  and
///        output buffers.
/// @param buff       -- Buffer to initialize
/// @param buff_size  -- Size of the buffer
/// @return Initialized buffer for the filter.
////////////////////////////////////////////////////////////
void Filter::initBuffer(float* buff, unsigned int buff_size){
	for (unsigned int i=0; i<buff_size; ++i )
	{
		buff[i] = 0.0;
	}
}

////////////////////////////////////////////////////////////
/// @brief Generic Discrete filter function. Be sure to
///        properly configure the filter before using.
///        See the FilterConfiguration class and the filter
///        initialization function.
/// @param inputValue  -- Input to the filter.
/// @return Output from the filter
////////////////////////////////////////////////////////////
float Filter::filter(float inputValue) {
  float outputContribution = 0.0;
  float inputContribution = 0.0;
  /// Buffer the input values and previous input values.
  for(unsigned int i=_numInWeights; i>0; i--){
	_inputBuffer[i] = _inputBuffer[i-1];
  }
  _inputBuffer[0] = inputValue;
  /// Buffer the output values and previous output values.
  for(unsigned int i=_numOutWeights; i>0; i--){
	_outputBuffer[i] = _outputBuffer[i-1];
  }
  for(unsigned int i=0; i<_numInWeights; i++){
    inputContribution += _inputWeights[i]*_inputBuffer[i];
  }
  for(unsigned int i=1; i<_numOutWeights; i++){
    outputContribution += _outputWeights[i]*_outputBuffer[i];
  }
  /// @note calculate the current filter output based on previous
  ///       inputs and previous outputs.
  _outputBuffer[0] = (1.0/_outputWeights[0])*(inputContribution + outputContribution);
  return _outputBuffer[0];
}
