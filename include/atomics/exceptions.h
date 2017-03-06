#ifndef _ATOMICS_EXCEPTIONS_H_
#define _ATOMICS_EXCEPTIONS_H_

#include <exception>

namespace atomics
{

class TryLockException : public std::exception
{
public:
    virtual char const * what()
    {
        return "Failed to acquire lock";
    }
};

} // atomic

#endif //_ATOMICS_EXCEPTIONS_H_
