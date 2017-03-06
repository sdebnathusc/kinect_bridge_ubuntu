#ifndef _ATOMICS_MEMORYPOOLALLOCATOR_H_
#define _ATOMICS_MEMORYPOOLALLOCATOR_H_

#include <Poco/MemoryPool.h>
// for std::bad_alloc
#include <new>
// for std::max
#include <algorithm>

#include <atomics/wrapper.h>
#include <atomics/print.h>

//#include <iostream>

namespace atomics
{

template<class __Data>
struct MemoryPoolAllocator
{
    typedef MemoryPoolAllocator<__Data> _MemoryPoolAllocator;

    typedef __Data value_type;
    typedef ::Poco::MemoryPool _MemPool;
    typedef ::atomics::Wrapper<_MemPool> _MemPoolWrapper;

    ::atomics::Wrapper<_MemPool> & mem_pool_;
    __Data * mem_block_;
    // size in bytes
    size_t size_bytes_;

    // default
    MemoryPoolAllocator( _MemPoolWrapper & mem_pool )
    :
        mem_pool_( mem_pool ),
        mem_block_( NULL ),
        size_bytes_( 0 )
    {
//        atomics_print( "MemoryPoolAllocator default" << std::endl );
    }

    // copy, convert
    template<class __OtherData>
    MemoryPoolAllocator( MemoryPoolAllocator<__OtherData> const & other )
    :
        mem_pool_( other.mem_pool_ ),
        mem_block_( NULL ),
        size_bytes_( other.size_bytes_ )
    {
//        atomics_print( "MemoryPoolAllocator copy, convert" << std::endl );
        auto mem_pool_handle = mem_pool_.getHandle();
        init( mem_pool_handle );
    }

    // copy
    MemoryPoolAllocator( _MemoryPoolAllocator const & other )
    :
        mem_pool_( other.mem_pool_ ),
        mem_block_( NULL ),
        size_bytes_( other.size_bytes_ )
    {
//        atomics_print( "MemoryPoolAllocator copy" << std::endl );
        auto mem_pool_handle = mem_pool_.getHandle();
        init( mem_pool_handle );
    }

    // move, convert
    template<class __OtherData>
    MemoryPoolAllocator( MemoryPoolAllocator<__OtherData> && other )
    :
        mem_pool_( other.mem_pool_ ),
        mem_block_( reinterpret_cast<__Data *>( other.mem_block_ ) ),
        size_bytes_( other.size_bytes_ )
    {
//        atomics_print( "MemoryPoolAllocator move, convert" << std::endl );
        if( reinterpret_cast<_MemoryPoolAllocator*>( &other ) != this )
        {
            other.mem_block_ = NULL;
            other.size_bytes_ = 0;
        }
    }

    // move
    MemoryPoolAllocator( _MemoryPoolAllocator && other )
    :
        mem_pool_( other.mem_pool_ ),
        mem_block_( other.mem_block_ ),
        size_bytes_( other.size_bytes_ )
    {
//        atomics_print( "MemoryPoolAllocator move" << std::endl );
        if( reinterpret_cast<_MemoryPoolAllocator*>( &other ) != this )
        {
            other.mem_block_ = NULL;
            other.size_bytes_ = 0;
        }
    }

    ~MemoryPoolAllocator()
    {
        deallocate();
    }

    void release( _MemPoolWrapper::_Handle & mem_pool_handle )
    {
        if( mem_block_ )
        {
//            atomics_print( "releasing block " << static_cast<void*>( mem_block_ ) << " back to pool" << std::endl );
            mem_pool_handle->release( mem_block_ );
        }
        size_bytes_ = 0;
        mem_block_ = NULL;
    }

    void deallocate()
    {
        auto mem_pool_handle = mem_pool_.getHandle();
        release( mem_pool_handle );
    }

    void init( _MemPoolWrapper::_Handle & mem_pool_handle )
    {
        if( !mem_block_ )
        {
//            atomics::print( "fetching mem block from pool" << std::endl );
            mem_block_ = reinterpret_cast<__Data *>( mem_pool_handle->get() );
//            atomics_print( "acquired block " << static_cast<void*>( mem_block_ ) << " from pool" << std::endl );
        }
    }

    __Data * begin()
    {
        return mem_block_;
    }

    __Data const * begin() const
    {
        return mem_block_;
    }

    __Data * allocate( size_t num_items )
    {
        size_t const num_bytes = num_items * sizeof( __Data );

//        atomics::print( "allocating " << num_items << " (" << num_bytes << ") items" << std::endl );

        auto mem_pool_handle = mem_pool_.getHandle();

        // if there's room for the requested items
        if( maxSizeBytes() - sizeBytes() >= num_bytes )
        {
            // make sure we have a memory block
            init( mem_pool_handle );

            // calculate where in our memory block the returned items begin
            __Data * target_block = mem_block_ + size();

            // update our overall size
            size_bytes_ += num_bytes;

            return target_block;
        }

        // if we don't have enough room for the requested items
        throw std::bad_alloc();
    }

    void deallocate( __Data * data, size_t const & num_items )
    {
        size_t const num_bytes = num_items * sizeof( __Data );

//        atomics::print( "deallocating " << num_items << " (" << num_bytes << ") items" << std::endl );

        auto mem_pool_handle = mem_pool_.getHandle();

        // if we're being asked to deallocate all (or more) than we are currently holding
        if( num_bytes >= sizeBytes() )
        {
            // release our memory block back to the pool
            release( mem_pool_handle );
        }
        else
        {
            // update our overall size
            size_bytes_ -= num_bytes;
        }
    }

    size_t size() const
    {
//        return std::max( sizeBytes() / sizeof( __Data ), (size_t)1 );
        return sizeBytes() / sizeof( __Data );
    }

    size_t sizeBytes() const
    {
        return size_bytes_;
    }

    size_t max_size() const
    {
//        return std::max( maxSizeBytes() / sizeof( __Data ), (size_t)1 );
        return maxSizeBytes() / sizeof( __Data );
    }

    size_t maxSizeBytes() const
    {
        return mem_pool_->blockSize();
    }
};

template<class __Data1, class __Data2>
bool operator==( MemoryPoolAllocator<__Data1> const & allocator1, MemoryPoolAllocator<__Data2> const & allocator2 )
{
    return allocator1.mem_block_ == allocator2.mem_block_;
}

template<class __Data1, class __Data2>
bool operator!=( MemoryPoolAllocator<__Data1> const & allocator1, MemoryPoolAllocator<__Data2> const & allocator2 )
{
    return allocator1.mem_block_ != allocator2.mem_block_;
}


} // atomics

#endif // _ATOMICS_MEMORYPOOLALLOCATOR_H_
