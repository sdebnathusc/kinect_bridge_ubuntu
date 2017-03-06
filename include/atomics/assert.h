#ifndef _ATOMICS_ASSERT_H_
#define _ATOMICS_ASSERT_H_

#include <atomics/print.h>
#include <assert.h>

#define runtime_assert( value, message ) \
    if( !value ) atomics_print( message << std::endl ); \
    assert( value );

#define runtime_assert_true( actual, desired, message ) \
    if( actual != desired ) atomics_print( message << " (" << actual << ")" << std::endl ); \
    assert( actual == desired );

#define runtime_assert_false( actual, desired, message ) \
    if( actual == desired ) atomics_print( message << " (" << actual << ")" << std::endl ); \
    assert( actual != desired );

#endif // _ATOMICS_ASSERT_H_
