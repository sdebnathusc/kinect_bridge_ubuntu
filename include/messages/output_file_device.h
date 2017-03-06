#ifndef _MESSAGES_OUTPUTFILEDEVICE_H_
#define _MESSAGES_OUTPUTFILEDEVICE_H_

#include <messages/container_messages.h>
#include <messages/binary_message.h>

#include <fstream>

class OutputFileDeviceMessageHeader : public RecursiveMessageHeader
{
public:
    std::string output_path_;
    std::ios_base::openmode output_mode_;

    OutputFileDeviceMessageHeader( std::string const & output_path = "", std::ios_base::openmode output_mode = std::ios_base::openmode() )
    :
        output_path_( output_path ),
        output_mode_( output_mode )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        RecursiveMessageHeader::pack( archive );
        archive << output_path_;
        archive << output_mode_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        RecursiveMessageHeader::unpack( archive );
        archive >> output_path_;
        archive >> output_mode_;
    }
};

template<class __Payload = BinaryMessage<> >
class OutputFileDeviceMessage : public SerializableMessageInterface<OutputFileDeviceMessageHeader, __Payload>
{
public:
    typedef __Payload _Payload;
    typedef SerializableMessageInterface<OutputFileDeviceMessageHeader, __Payload> _Message;

    template<class... __Args>
    OutputFileDeviceMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    DECLARE_MESSAGE_INFO( OutputFileDeviceMessage )
};

class OutputFileDevice
{
public:
    typedef std::ofstream _OutputStream;

    _OutputStream output_stream_;
    OutputFileDeviceMessageHeader output_state_;

    OutputFileDevice() = default;

    template<class __Head, class... __Args>
    OutputFileDevice( __Head && head, __Args&&... args )
    :
        output_stream_( std::forward<__Head>( head ), std::forward<__Args>( args )... )
    {
        updateOutputState( std::forward<__Head>( head ), std::forward<__Args>( args )... );
    }

    template<class... __Args>
    void openOutput( __Args&&... args )
    {
        output_stream_.open( std::forward<__Args>( args )... );
        updateOutputState( std::forward<__Args>( args )... );
    }

    template<class... __Args>
    void updateOutputState( __Args&&... args )
    {
        auto args_tuple = std::make_tuple( std::forward<__Args>( args )... );
        output_state_.output_path_ = std::get<0>( args_tuple );
        output_state_.output_mode_ = std::get<1>( args_tuple );
    }

    void closeOutput()
    {
        output_stream_.close();
    }

    template<class __Serializable>
    void push( __Serializable & serializable )
    {
        serializable.pack( output_stream_ );
    }

    template<class __Payload>
    void push( OutputFileDeviceMessage<__Payload> & device_message )
    {
        auto & header = device_message.header_;
        auto & payload = device_message.payload_;

        openOutput( header.output_path_, header.output_mode_ );

        push( payload );
    }
};

#endif // _MESSAGES_OUTPUTFILEDEVICE_H_
