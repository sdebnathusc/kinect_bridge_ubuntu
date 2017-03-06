#ifndef _ATOMICS_WRAPPER_H_
#define _ATOMICS_WRAPPER_H_

#include <mutex>
#include <condition_variable>
#include <atomic>

#include <atomics/handle.h>

namespace atomics
{

template<class __Data, class __Mutex = std::timed_mutex>
class Wrapper
{
public:
    typedef __Data _Data;
    typedef __Mutex _Mutex;

    typedef std::condition_variable_any _Condition;
    typedef Handle<Wrapper<__Data> > _Handle;

    __Data data_;

    __Mutex mutex_;
    _Condition condition_;
    __Mutex condition_mutex_;

    Wrapper()
    :
        data_()
    {
        //
    }

    Wrapper( __Data const & data )
    :
        data_( data )
    {
        //
    }

    template<class... __Args>
    Wrapper( __Args&&... args )
    :
        data_( std::forward<__Args>( args )... )
    {
        //
    }

    _Handle getHandle( HandleBase::NotifyType policy = HandleBase::NotifyType::DELAY_NOTIFY_NONE )
    {
        return _Handle( this, policy );
    }

    __Mutex & getMutex()
    {
        return mutex_;
    }

    __Mutex const & getMutex() const
    {
        return mutex_;
    }

    _Condition & getCondition()
    {
        return condition_;
    }

    _Condition const & getCondition() const
    {
        return condition_;
    }

    __Data * operator->()
    {
        return &data_;
    }

    __Data const * operator->() const
    {
        return &data_;
    }

    __Data & operator*()
    {
        return data_;
    }

    __Data const & operator*() const
    {
        return data_;
    }
};

} // atomic

#endif //_ATOMICS_WRAPPER_H_
