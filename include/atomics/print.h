#ifndef _ATOMICS_PRINT_H_
#define _ATOMICS_PRINT_H_

#include <mutex>
#include <iostream>

namespace atomics
{

static std::mutex cout_mutex__;

}

#define atomics_print( ... ) { std::lock_guard<std::mutex> __ATOMICSPRINTLOCK__( atomics::cout_mutex__ ); std::cout << __VA_ARGS__; }

#endif // _ATOMICS_PRINT_H_
