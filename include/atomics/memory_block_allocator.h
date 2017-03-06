#ifndef _ATOMICS_MEMORYBLOCKALLOCATOR_H_
#define _ATOMICS_MEMORYBLOCKALLOCATOR_H_

#include <Poco/Buffer.h>
// for std::bad_alloc
#include <new>
// for std::max
#include <algorithm>

#include <iostream>

namespace atomics
{

template<class __Data>
struct MemoryBlockAllocator
{
    typedef __Data value_type;
    typedef ::Poco::Buffer<char> _MemBlock;

    _MemBlock & mem_block_;
    // size in bytes
    size_t size_bytes_;

    MemoryBlockAllocator( _MemBlock & mem_block )
    :
        mem_block_( mem_block ),
        size_bytes_( 0 )
    {
        //
    }

    template<class __OtherData>
    MemoryBlockAllocator( MemoryBlockAllocator<__OtherData> const & other )
    :
        mem_block_( other.mem_block_ ),
        size_bytes_( other.size_bytes_ )
    {
        //
    }

    __Data * allocate( size_t num_items )
    {
        size_t const num_bytes = num_items * sizeof( __Data );

        std::cout << "allocating " << num_items << " (" << num_bytes << ") items" << std::endl;

        // if enough space has been allocated
        if( maxSizeBytes() - sizeBytes() >= num_bytes )
        {
            __Data * target_block = reinterpret_cast<__Data *>( mem_block_.begin() + sizeBytes() );
            size_bytes_ += num_bytes;

            return target_block;
        }

        throw std::bad_alloc();
    }

    void deallocate( __Data * data, size_t const & num_items )
    {
        size_t const num_bytes = num_items * sizeof( __Data );

        std::cout << "deallocating " << num_items << " (" << num_bytes << ") items" << std::endl;

        if( num_bytes >= sizeBytes() )
        {
            size_bytes_ = 0;
        }
        else
        {
            size_bytes_ -= num_bytes;
        }
    }

    size_t size() const
    {
        return std::max( sizeBytes() / sizeof( __Data ), (size_t)1 );
    }

    size_t sizeBytes() const
    {
        return size_bytes_;
    }

    size_t max_size() const
    {
        return std::max( maxSizeBytes() / sizeof( __Data ), (size_t)1 );
    }

    size_t maxSizeBytes() const
    {
        return mem_block_.capacity();
    }
};

template<class __Data1, class __Data2>
bool operator==( MemoryBlockAllocator<__Data1> const & allocator1, MemoryBlockAllocator<__Data2> const & allocator2 )
{
    return allocator1.mem_block_.begin() == allocator2.mem_block_.begin();
}

template<class __Data1, class __Data2>
bool operator!=( MemoryBlockAllocator<__Data1> const & allocator1, MemoryBlockAllocator<__Data2> const & allocator2 )
{
    return allocator1.mem_block_.begin() != allocator2.mem_block_.begin();
}


} // atomics

#endif // _ATOMICS_MEMORYBLOCKALLOCATOR_H_
