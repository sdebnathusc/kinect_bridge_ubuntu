#ifndef _ATOMICS_HANDLE_H_
#define _ATOMICS_HANDLE_H_

#include <mutex>
#include <condition_variable>
#include <iostream>
#include <string>
#include <atomic>
#include <cstdint>

#include <atomics/exceptions.h>
//#include <atomics/print.h>

namespace atomics
{

class HandleBase
{
public:
    enum class NotifyType
    {
        DELAY_NOTIFY_NONE = 0x00,
        DELAY_NOTIFY_ONE = 0x01,
        DELAY_NOTIFY_ALL = 0x02
    };
};

template<class __Wrapper>
class Handle : public HandleBase
{
public:
    typedef typename __Wrapper::_Data _Data;
    typedef typename __Wrapper::_Mutex _Mutex;

    typedef Handle<__Wrapper> _Handle;

    typedef std::unique_lock<_Mutex> _Lock;

protected:
    std::atomic<uint8_t> _notify_destruct;
    std::atomic<uint8_t> _notify_policy;

public:
    __Wrapper * wrapper_;
    _Lock lock_;
    std::string name_;

/*
    Handle( _Handle const & other )
    {
        if( &other != this )
        {
            *this = other;
        }
    }

    void operator=( _Handle const & other )
    {
        if( &other != this )
        {
            *this = other;
        }
    }
*/

/*
    Handle( _Handle const & other )
    :
        _notify_destruct( other._notify_destruct_.load() ),
        _notify_policy( other._notify_policy.load() ),
        wrapper_( other.wrapper_ ),
        lock_( other.lock_ )
    {
//        std::cout << "handle copy constructor" << std::endl;
    }
*/

    Handle( _Handle && other )
    :
        _notify_destruct( other._notify_destruct.load() ),
        _notify_policy( other._notify_policy.load() ),
        wrapper_( other.wrapper_ ),
        lock_( std::move( other.lock_ ) )
    {
//        std::cout << "handle move constructor" << std::endl;
    }

    Handle( __Wrapper * wrapper, HandleBase::NotifyType notify_policy = NotifyType::DELAY_NOTIFY_NONE )
    :
        _notify_destruct( static_cast<uint8_t>( NotifyType::DELAY_NOTIFY_NONE ) ),
        _notify_policy( static_cast<uint8_t>( notify_policy ) ),
        wrapper_( wrapper ),
        lock_( wrapper->mutex_, std::defer_lock )
    {
//        std::cout << "handle default constructor" << std::endl;
    }

    // send requested notifications on destruction (ensure mutex is unlocked first)
    ~Handle()
    {
//        if( !name_.empty() ) std::cout << "Handle [" << name_ << "] going out of scope" << std::endl;

        release();

        if( _notify_destruct & static_cast<uint8_t>( NotifyType::DELAY_NOTIFY_ALL ) )
        {
            if( !name_.empty() ) std::cout << "Handle [" << name_ << "] notifying all" << std::endl;
            notifyAll();
        }
        else if( _notify_destruct & static_cast<uint8_t>( NotifyType::DELAY_NOTIFY_ONE ) )
        {
            if( !name_.empty() ) std::cout << "Handle [" << name_ << "] notifying one" << std::endl;
            notifyOne();
        }

//        if( !name_.empty() ) std::cout << "Handle [" << name_ << "] done going out of scope" << std::endl;
    }

    void release()
    {
        try
        {
//            if( !name_.empty() ) std::cout << "Handle [" << name_ << "] unlocking" << std::endl;

            // MSVCP 12 : no exception thrown for unlocking an unlocked lock
            // gcc 4.8.1 : throws std::errc::operation_not_permitted
            lock_.unlock();
//            if( !name_.empty() ) std::cout << "Handle [" << name_ << "] done unlocking" << std::endl;
        }
        catch( std::system_error & e )
        {
            if( e.code() != std::errc::operation_not_permitted ) throw e;
//            if( !name_.empty() ) std::cout << "Handle [" << name_ << "] not locked" << std::endl;
        }
    }

    _Lock & getLock()
    {
        return lock_;
    }

    _Lock const & getLock() const
    {
        return lock_;
    }

    // read-only
    _Data const & get() const
    {
        return wrapper_->data_;
    }

    // read/write
    _Data & get()
    {
        return wrapper_->data_;
    }

    // read/write
    _Data & getExclusive()
    {
        try
        {
            // MSVCP 12 : throws std::errc::device_or_resource_busy
            // gcc 4.8.1 : throws std::errc::operation_not_permitted
            if( !lock_ ) lock_.lock();
        }
        catch( std::system_error & e )
        {
#ifdef _WIN32
            if( e.code() != std::errc::device_or_resource_busy )
#else
            if( e.code() != std::errc::resource_deadlock_would_occur )
#endif
            {
                if( !name_.empty() ) std::cout << "Handle [" << name_ << "] locking error" << std::endl;
                std::cout << "!!! Locking error code " << e.code() << ": " << e.what() << std::endl;
                throw e;
            }
            // already locked by this thread; that's fine
        }
        return delayNotify().get();
    }

    _Data & tryGetExclusive()
    {
        bool is_locked = false;
        try
        {
            // MSVCP 12 : throws std::errc::device_or_resource_busy
            // gcc 4.8.1 : throws std::errc::operation_not_permitted
            is_locked = lock_ || lock_.try_lock();
        }
        catch( std::system_error & e )
        {
#ifdef _WIN32
            if( e.code() != std::errc::device_or_resource_busy )
#else
            if( e.code() != std::errc::resource_deadlock_would_occur )
#endif
            {
                throw e;
            }
            // already locked by this thread; that's fine
            is_locked = true;
        }

        if( is_locked ) return delayNotify().get();

        // failed; already locked by another thread
        throw atomics::TryLockException();
    }

    _Data & tryGetExclusiveFor( std::chrono::high_resolution_clock::duration const & duration )
    {
        bool is_locked = false;
        try
        {
            // MSVCP 12 : throws std::errc::device_or_resource_busy
            // gcc 4.8.1 : throws std::errc::operation_not_permitted
            is_locked = lock_ || lock_.try_lock_for( duration );
        }
        catch( std::system_error & e )
        {
#ifdef _WIN32
            if( e.code() != std::errc::device_or_resource_busy )
#else
            if( e.code() != std::errc::resource_deadlock_would_occur )
#endif
            {
                throw e;
            }
            // already locked by this thread; that's fine
            is_locked = true;
        }

        if( is_locked ) return delayNotify().get();

//        std::cout << "tryGetExclusiveFor() timed out" << std::endl;

        // failed; locked by another thread for too long
        throw atomics::TryLockException();
    }

/*
    _Handle & waitOn()
    {
        release();

        _Lock lock( wrapper_->condition_mutex_ );
        wrapper_->condition_.wait( lock );
        return *this;
    }
*/

    _Handle & waitOn()
    {
//        if( !name_.empty() ) std::cout << "Handle [" << name_ << "] waitOn()" << std::endl;
        try
        {
            // MSVCP 12 : throws std::errc::device_or_resource_busy
            // gcc 4.8.1 : throws std::errc::operation_not_permitted
            if( !lock_ ) lock_.lock();
//            if( !name_.empty() ) std::cout << "Handle [" << name_ << "] locked" << std::endl;
        }
        catch( std::system_error & e )
        {
#ifdef _WIN32
            if( e.code() != std::errc::device_or_resource_busy )
#else
            if( e.code() != std::errc::resource_deadlock_would_occur )
#endif
            {
                if( !name_.empty() ) std::cout << "Handle [" << name_ << "] locking error" << std::endl;
                std::cout << "!!! Locking error code " << e.code() << ": " << e.what() << std::endl;
                throw e;
            }
            if( !name_.empty() ) std::cout << "Handle [" << name_ << "] already locked" << std::endl;

            // already locked by this thread; that's fine
        }

//        if( !name_.empty() ) std::cout << "Handle [" << name_ << "] waiting on condition" << std::endl;
//        _Lock lock( wrapper_->condition_mutex_ );
        wrapper_->condition_.wait( lock_ );
        return *this;
    }

/*
    _Handle & waitOnFor( std::chrono::high_resolution_clock::duration const & duration )
    {
        auto const start_time = std::chrono::high_resolution_clock::now();
        auto const end_time = start_time + duration;

        // we can't hold the lock on the data while we wait, otherwise nobody would be able to modify the data before notifying us
        release();

        _Lock lock( wrapper_->condition_mutex_, end_time );
        if( lock && std::chrono::high_resolution_clock::now() < end_time && wrapper_->condition_.wait_until( lock, end_time ) == std::cv_status::no_timeout )
        {
            return *this;
        }

        // failed; locked by another thread for too long
        throw atomics::TryLockException();
    }
*/

    _Handle & waitOnFor( std::chrono::high_resolution_clock::duration const & duration )
    {
        auto const start_time = std::chrono::high_resolution_clock::now();
        auto const end_time = start_time + duration;

        bool is_locked = false;
        try
        {
            // MSVCP 12 : throws std::errc::device_or_resource_busy
            // gcc 4.8.1 : throws std::errc::operation_not_permitted
            is_locked = lock_.try_lock_until( end_time );
        }
        catch( std::system_error & e )
        {
#ifdef _WIN32
            if( e.code() != std::errc::device_or_resource_busy )
#else
            if( e.code() != std::errc::resource_deadlock_would_occur )
#endif
            {
                throw e;
            }

            // already locked by this thread; that's fine
            is_locked = true;
        }

        if( is_locked && std::chrono::high_resolution_clock::now() < end_time && wrapper_->condition_.wait_until( lock_, end_time ) == std::cv_status::no_timeout )
        {
            return *this;
        }

        release();

        // failed; locked by another thread for too long
        throw atomics::TryLockException();
    }

    void notifyOne()
    {
//        std::cout <<  "AtomicHandle notifying one [" << name_ << "]" << std::endl;
        wrapper_->condition_.notify_one();
    }

    void notifyAll()
    {
//        std::cout <<  "AtomicHandle notifying all [" << name_ << "]" << std::endl;
        wrapper_->condition_.notify_all();
    }

    _Handle & delayNotify()
    {
        _notify_destruct |= _notify_policy;
        return *this;
    }

    _Handle & delayNotifyOne()
    {
        _notify_destruct |= NotifyType::DELAY_NOTIFY_ONE;
        return *this;
    }

    _Handle & delayNotifyAll()
    {
        _notify_destruct |= NotifyType::DELAY_NOTIFY_ALL;
        return *this;
    }

    // write access
    _Data * operator->()
    {
        return &getExclusive();
    }

    // read only access
    _Data const * operator->() const
    {
        return &get();
    }

    // write access
    _Data & operator*()
    {
        return getExclusive();
    }

    // read only access
    _Data const & operator*() const
    {
        return get();
    }
};

} // atomic

#endif //_ATOMICS_HANDLE_H_
