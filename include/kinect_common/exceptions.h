#ifndef _KINECTCOMMON_EXCEPTIONS_H_
#define _KINECTCOMMON_EXCEPTIONS_H_

#include <stdexcept>

class KinectException : public std::runtime_error
{
public:
    template<class... __Args>
    KinectException( __Args&&... args )
    :
        std::runtime_error( std::forward<__Args>( args )... )
    {
        //
    }
};

#endif //_KINECTCOMMON_EXCEPTIONS_H_
