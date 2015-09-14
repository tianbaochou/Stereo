#ifndef STEREO_STEREO_STEREO_ERROR_H
#define STEREO_STEREO_STEREO_ERROR_H
#include <stdexcept>
using std::exception;

class StereoException : public exception{
public:
    StereoException(const char * message) :exception(message){ }
};


#endif