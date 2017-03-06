#ifndef _MESSAGES_INPUTFILEDEVICE_H_
#define _MESSAGES_INPUTFILEDEVICE_H_

#include <messages/container_messages.h>
#include <messages/binary_message.h>

#include <fstream>

class InputFileDeviceMessageHeader : public RecursiveMessageHeader
{
public:
    std::string input_path_;
    std::ios_base::openmode input_mode_;

    InputFileDeviceMessageHeader( std::string const & input_path = "", std::ios_base::openmode input_mode = std::ios_base::openmode() )
    :
        input_path_( input_path ),
        input_mode_( input_mode )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        RecursiveMessageHeader::pack( archive );
        archive << input_path_;
        archive << input_mode_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        RecursiveMessageHeader::unpack( archive );
        archive >> input_path_;
        archive >> input_mode_;
    }
};

template<class __Payload = BinaryMessage<> >
class InputFileDeviceMessage : public SerializableMessageInterface<InputFileDeviceMessageHeader, __Payload>
{
public:
    typedef __Payload _Payload;
    typedef SerializableMessageInterface<InputFileDeviceMessageHeader, __Payload> _Message;

    template<class... __Args>
    InputFileDeviceMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    DECLARE_MESSAGE_INFO( InputFileDeviceMessage )
};

class InputFileDevice
{
public:
    typedef std::ifstream _InputStream;

    _InputStream input_stream_;
    InputFileDeviceMessageHeader input_state_;

    InputFileDevice() = default;

    template<class __Head, class... __Args>
    InputFileDevice( __Head && head, __Args&&... args )
    :
        input_stream_( std::forward<__Head>( head ), std::forward<__Args>( args )... )
    {
        updateInputState( std::forward<__Head>( head ), std::forward<__Args>( args )... );
    }

    template<class... __Args>
    void updateInputState( __Args&&... args )
    {
        auto args_tuple = std::make_tuple( std::forward<__Args>( args )... );
        input_state_.input_path_ = std::get<0>( args_tuple );
        input_state_.input_mode_ = std::get<1>( args_tuple );
    }

    template<class... __Args>
    void openInput( __Args&&... args )
    {
        input_stream_.open( std::forward<__Args>( args )... );
        updateInputState( std::forward<__Args>( args )... );
    }

    void closeInput()
    {
        input_stream_.close();
    }

    template<class __Serializable>
    void pull( __Serializable & serializable )
    {
        serializable.unpack( input_stream_ );
    }

    template<class __Serializable>
    __Serializable pullAs()
    {
        __Serializable serializable;
        pull( serializable );
        return serializable;
    }

    template<class __Payload>
    InputFileDeviceMessage<__Payload> pullDeviceMessage()
    {
        return InputFileDeviceMessage<__Payload>( InputFileDeviceMessageHeader( input_state_ ), pull<__Payload>() );
    }

    template<class __Payload>
    __Payload pullFrom( InputFileDeviceMessage<__Payload> const & device_message )
    {
        openInput( device_message.header_.input_path_, device_message.header_.input_mode_ );
        return pullAs<__Payload>();
    }

    template<class __Payload>
    void pullDeviceMessage( InputFileDeviceMessage<__Payload> & device_message )
    {
        openInput( device_message.header_.input_path_, device_message.header_.input_mode_ );
        device_message.header_ = input_state_;
        pull( device_message.payload_ );
    }
};

#endif // _MESSAGES_INPUTFILEDEVICE_H_
