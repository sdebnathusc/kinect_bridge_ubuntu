#ifndef _MESSAGES_IOFILEDEVICE_H_
#define _MESSAGES_IOFILEDEVICE_H_

#include <messages/input_file_device.h>
#include <messages/output_file_device.h>

class IOFileDeviceMessageHeader : public InputFileDeviceMessageHeader, public OutputFileDeviceMessageHeader
{
public:
    template<class __Archive>
    void pack( __Archive & archive )
    {
        InputFileDeviceMessageHeader::pack( archive );
        OutputFileDeviceMessageHeader::pack( archive );
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        InputFileDeviceMessageHeader::unpack( archive );
        OutputFileDeviceMessageHeader::unpack( archive );
    }
};

template<class __Payload = BinaryMessage<> >
class IOFileDeviceMessage : public SerializableMessageInterface<IOFileDeviceMessageHeader, __Payload>
{
public:
    typedef __Payload _Payload;
    typedef SerializableMessageInterface<IOFileDeviceMessageHeader, __Payload> _Message;

    template<class... __Args>
    IOFileDeviceMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    DECLARE_MESSAGE_INFO( IOFileDeviceMessage )
};

class IOFileDevice : public InputFileDevice, public OutputFileDevice
{
public:
    typedef InputFileDevice::_InputStream _InputStream;
    typedef OutputFileDevice::_OutputStream _OutputStream;

    IOFileDevice() = default;

    template<class... __InputArgs, class... __OutputArgs>
    IOFileDevice( __InputArgs&&... input_args, __OutputArgs&&... output_args )
    :
        InputFileDevice( std::forward<__InputArgs>( input_args )... ),
        OutputFileDevice( std::forward<__OutputArgs>( output_args )... )
    {
        //
    }
};

#endif // _MESSAGES_IOFILEDEVICE_H_
