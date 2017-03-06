#ifndef _ATOMICS_ARRAY_H_
#define _ATOMICS_ARRAY_H_

namespace atomics
{

template<class __Data = char>
class Array
{
public:
    __Data * data_;
    size_t size_;

    Array( __Data * data = NULL, size_t const & size = 0 )
    :
        data_( data ),
        size_( size )
    {
        //
    }

    __Data * begin()
    {
        return data_;
    }

    __Data const * begin() const
    {
        return data_;
    }

    size_t const & size() const
    {
        return size_;
    }
};

} // atomics


#endif // _ATOMICS_ARRAY_H_
