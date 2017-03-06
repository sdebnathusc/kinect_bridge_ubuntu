#ifndef _KINECTCOMMON_MEMORY_H_
#define _KINECTCOMMON_MEMORY_H_

#include <memory>
#include <iostream>

// Safe release for interfaces
template<class __Interface>
inline void SafeRelease( __Interface *& pInterfaceToRelease )
{
    if( pInterfaceToRelease != NULL )
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

// ####################################################################################################
template<class __Wrapped>
class ReleasableWrapper
{
public:
    typedef __Wrapped _Wrapped;
    typedef __Wrapped * _WrappedPtr;
    typedef __Wrapped const * _WrappedConstPtr;

    typedef uint8_t _TrackerData;
    typedef std::shared_ptr<_TrackerData> _Tracker;

    _Tracker tracker_;

    _WrappedPtr wrapped_ptr_;

    // ====================================================================================================
    ReleasableWrapper( _WrappedPtr wrapped_ptr = NULL )
    :
        wrapped_ptr_( wrapped_ptr ),
        tracker_( std::make_shared<_TrackerData>() )
    {
        //
    }

    // ====================================================================================================
    ~ReleasableWrapper()
    {
        release();
    }

    // ====================================================================================================
    void release()
    {
        if( wrapped_ptr_ && tracker_.unique() )
        {
            wrapped_ptr_->Release();
            wrapped_ptr_ = NULL;
        }
    }

    // ====================================================================================================
    _WrappedPtr operator->()
    {
        return wrapped_ptr_;
    }

    // ====================================================================================================
    _WrappedConstPtr operator->() const
    {
        return const_cast<_WrappedConstPtr>( wrapped_ptr_ );
    }

    // ====================================================================================================
    _WrappedPtr & get()
    {
        return wrapped_ptr_;
    }

    // ====================================================================================================
    _WrappedConstPtr get() const
    {
        return wrapped_ptr_;
    }

    __Wrapped ** getAddr()
    {
        return &wrapped_ptr_;
    }
};


#endif // _KINECTCOMMON_MEMORY_H_
