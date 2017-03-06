#ifndef _ATOMICS_MEMPOOLBUFFER_H_
#define _ATOMICS_MEMPOOLBUFFER_H_

#include <Poco/MemoryPool.h>
#include <atomics/wrapper.h>

//#include <iostream>

namespace atomics
{

template<class __Data>
class MemPoolBuffer
{
public:
    typedef Poco::MemoryPool _MemPool;
    typedef __Data _Data;
    typedef atomics::Wrapper<_MemPool> _MemPoolWrapper;

    _MemPoolWrapper * mem_pool_ptr_;
    // size in __Data units
    size_t size_;
    __Data * data_ptr_;

    MemPoolBuffer( _MemPoolWrapper & mem_pool, size_t const & size )
    :
        mem_pool_ptr_( &mem_pool ),
        size_( size ),
        data_ptr_( NULL )
    {
        //
    }

    ~MemPoolBuffer()
    {
        deallocate();
    }

    void reallocate( size_t size )
    {
        deallocate( size_ );
        allocate( size );
    }

    void reallocate()
    {
        reallocate( size_ );
    }

    void allocate( size_t size )
    {
//        std::cout << "allocating " << size << " items" << std::endl;

        auto mem_pool_handle = mem_pool_ptr_->getHandle();
        if( ( size + size_ ) * sizeof( __Data ) > mem_pool_handle->blockSize() ) throw std::bad_alloc();
        if( !data_ptr_ )
        {
//            std::cout << "fetching mem block from pool" << std::endl;
            data_ptr_ = reinterpret_cast<__Data *>( mem_pool_handle->get() );
        }
        size_ += size;
    }

    void allocate()
    {
        allocate( size_ );
    }

    void deallocate( size_t size )
    {
//        std::cout << "deallocating " << size << " items" << std::endl;

        auto mem_pool_handle = mem_pool_ptr_->getHandle();
        if( data_ptr_ && size >= size_ )
        {
//            std::cout << "releasing mem block back to pool" << std::endl;
            size_ = 0;
            mem_pool_handle->release( data_ptr_ );
            data_ptr_ = NULL;
        }
        else size_ -= size;
    }

    void deallocate()
    {
        deallocate( size_ );
    }

    __Data * get()
    {
        return data_ptr_;
    }

    __Data const * get() const
    {
        return data_ptr_;
    }

    size_t size() const
    {
        return size_;
    }

    size_t sizeBytes() const
    {
        return size_ * sizeof( __Data );
    }

    __Data * begin()
    {
        return data_ptr_;
    }

    __Data const * begin() const
    {
        return data_ptr_;
    }

    __Data * end()
    {
        return data_ptr_ + size_;
    }

    __Data const * end() const
    {
        return data_ptr_ + size_;
    }

    __Data * endContainer()
    {
        return data_ptr_ + (*mem_pool_ptr_)->blockSize();
    }

    __Data const * endContainer() const
    {
        return data_ptr_ + (*mem_pool_ptr_)->blockSize();
    }

    __Data & at( size_t const & index )
    {
        return *(begin() + index);
    }

    __Data const & at( size_t const & index ) const
    {
        return *(begin() + index);
    }

    __Data & operator[]( size_t const & index )
    {
        return at( index );
    }

    __Data const & operator[]( size_t const & index ) const
    {
        return at( index );
    }
};

} // atomics

#endif //_ATOMICS_MEMPOOLBUFFER_H_
