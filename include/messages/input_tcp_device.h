#ifndef _MESSAGES_INPUTTCPDEVICE_H_
#define _MESSAGES_INPUTTCPDEVICE_H_

#include <string>

#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketStream.h>
#include <Poco/MemoryStream.h>

#include <atomics/binary_stream.h>

#include <messages/container_messages.h>
#include <messages/binary_message.h>
#include <messages/exceptions.h>

#define INPUTTCPDEVICE_PROTOCOL_BUFSIZE 1024

class InputTCPDeviceMessageHeader : public RecursiveMessageHeader
{
public:
    std::string source_address_;
    uint16_t source_port_;

    InputTCPDeviceMessageHeader( std::string const & source_address = "localhost", uint16_t source_port = 0 )
    :
        source_address_( source_address ),
        source_port_( source_port )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        RecursiveMessageHeader::pack( archive );
        archive << source_address_;
        archive << source_port_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        RecursiveMessageHeader::unpack( archive );
        archive >> source_address_;
        archive >> source_port_;
    }
};

template<class __Payload = BinaryMessage<> >
class InputTCPDeviceMessage : public SerializableMessageInterface<InputTCPDeviceMessageHeader, __Payload>
{
public:
    typedef __Payload _Payload;
    typedef SerializableMessageInterface<InputTCPDeviceMessageHeader, __Payload> _Message;

    template<class... __Args>
    InputTCPDeviceMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    DECLARE_MESSAGE_INFO( InputTCPDeviceMessage )
};

class InputTCPDevice
{
public:
    typedef Poco::Net::SocketStream _InputStream;

    Poco::Net::StreamSocket input_socket_;
    InputTCPDeviceMessageHeader input_state_;

    char protocol_search_buf_[INPUTTCPDEVICE_PROTOCOL_BUFSIZE];

    InputTCPDevice() = default;

    // open the client socket and update our state
    template<class __Head, class... __Args>
    InputTCPDevice( __Head && head, __Args&&... args )
    :
        input_socket_( Poco::Net::SocketAddress( std::forward<__Head>( head ), std::forward<__Args>( args )... ) )
    {
        std::cout << "client connected on " << input_socket_.address().toString() << std::endl;
        updateInputState( std::forward<__Head>( head ), std::forward<__Args>( args )... );
    }

    ~InputTCPDevice()
    {
        closeInput();
    }

    template<class... __Args>
    void updateInputState( __Args&&... args )
    {
        auto args_tuple = std::make_tuple( std::forward<__Args>( args )... );
        input_state_.source_address_ = std::get<0>( args_tuple );
        input_state_.source_port_ = std::get<1>( args_tuple );
    }

    void openInput()
    {
        openInput( input_state_.source_address_, input_state_.source_port_ );
    }

    template<class __Head, class... __Args>
    void openInput( __Head&& head, __Args&&... args )
    {
        input_socket_ = Poco::Net::StreamSocket( Poco::Net::SocketAddress( std::forward<__Head>( head ), std::forward<__Args>( args )... ) );
        updateInputState( std::forward<__Head>( head ), std::forward<__Args>( args )... );
    }

    void closeInput()
    {
        try
        {
            // the server should be polling on reads to see if we're still connected; signal shutdown by closing our send channel
            std::cout << "sending shutdown to server" << std::endl;
            input_socket_.shutdownSend();

            // throw away any extra data on the line
            std::cout << "reading extra data from server" << std::endl;

            int receive_result = 0;
            do
            {
                receive_result = input_socket_.receiveBytes( protocol_search_buf_, INPUTTCPDEVICE_PROTOCOL_BUFSIZE );
                std::cout << "received: " << receive_result << std::endl;
            }
            while( receive_result > 0 );

            // now we can close the receive channel
            std::cout << "closing connection" << std::endl;
            input_socket_.shutdownReceive();
        }
        catch( std::exception & e )
        {
            // any exceptions we catch here just mean the connection is already shutdown and cleared
        }

        input_socket_.close();
    }

    template<class __Serializable>
    void pull( __Serializable & serializable )
    {
//        // make sure input stream is ready to receive
//        if( !client_stream_ )
//        {
//            // if not, see if the client socket is connected
//            if( input_socket_.impl()->initialized() )
//            {
//                // if the client is connected then we just need to connect the stream
//                client_stream_ = Poco::Net::SocketStream( input_socket_ );
//            }
//            // if the client isn't connected, we can't do anything; bail
//            else throw messages::MessageException( "Failed to deserialize message; TCPInputDevice not initialized" );
//        _}

        if( !input_socket_.impl()->initialized() ) throw messages::MessageException( "Failed to deserialize message; TCPInputDevice not initialized" );

        // assume next bytes are protocol message
//        std::memset( protocol_search_buf_, 0, INPUTTCPDEVICE_PROTOCOL_BUFSIZE );
        receiveBytes( input_socket_, protocol_search_buf_, 6 );

        bool msg_found = protocol_search_buf_[0] == '<' && protocol_search_buf_[5] == '>';
        size_t msg_begin = 0;

        uint32_t message_size = 0;

        // check for valid protocol message; if failed, read until we find another one
        while( !msg_found )
        {
            receiveBytes( input_socket_, protocol_search_buf_, INPUTTCPDEVICE_PROTOCOL_BUFSIZE );

            std::string protocol_search_str( protocol_search_buf_ );

            size_t start_point = 0;

            do
            {
                msg_begin = protocol_search_str.find( '<', start_point );
                if( msg_begin != std::string::npos )
                {
                    if( protocol_search_buf_[msg_begin + 5] == '>' )
                    {
                        msg_found = true;
                        break;
                    }

                    start_point = msg_begin + 1;
                }
            }
            while( start_point + 6 < INPUTTCPDEVICE_PROTOCOL_BUFSIZE );
        }
        size_t msg_end = msg_begin + 6;

        message_size = decodeMessageSize( protocol_search_buf_ + msg_begin );

        BinaryMessage<> raw_coded_message( NULL, message_size );
        raw_coded_message.allocate();

        // if the protocol message was found part-way through the receive buffer
        if( msg_begin > 0 )
        {
            // length of the message chunk we received (so everything after the protocol data)
            uint32_t received_chunk_length = INPUTTCPDEVICE_PROTOCOL_BUFSIZE - msg_end;

            // copy the rest of the input buffer into the raw message
            std::memcpy( raw_coded_message.data_, protocol_search_buf_ + msg_end, received_chunk_length );
            // receive the rest of the message
            receiveBytes( input_socket_, raw_coded_message.data_ + received_chunk_length, message_size - received_chunk_length );
        }
        else
        {
            receiveBytes( input_socket_, raw_coded_message.data_, raw_coded_message.size_ );
        }

        Poco::MemoryInputStream raw_input_stream( raw_coded_message.data_, raw_coded_message.size_ );
        atomics::BinaryInputStream binary_reader( raw_input_stream );
        binary_reader.readBOM();

        serializable.unpack( binary_reader );
    }

    uint32_t decodeMessageSize( char * protocol_message )
    {
        return *reinterpret_cast<uint32_t *>( protocol_message + 1 );
    }

    template<class __Socket>
    void receiveBytes( __Socket & socket, char * bytes, uint32_t length )
    {
        uint32_t bytes_received = 0;
        while( bytes_received < length )
        {
            int receive_result = socket.receiveBytes( bytes + bytes_received, length - bytes_received );
            if( receive_result < 0 ) throw messages::MessageException( "receiveBytes() failed" );
            else if( receive_result == 0 )
            {
                std::cout << "server disconnected" << std::endl;
                closeInput();
            }
            else bytes_received += receive_result;
        }
    }

    template<class __Serializable>
    __Serializable pullAs()
    {
        __Serializable serializable;
        pull( serializable );
        return serializable;
    }

    template<class __Payload>
    InputTCPDeviceMessage<__Payload> pullDeviceMessage()
    {
        return InputTCPDeviceMessage<__Payload>( InputTCPDeviceMessageHeader( input_state_ ), pull<__Payload>() );
    }

    template<class __Payload>
    __Payload pullFrom( InputTCPDeviceMessage<__Payload> const & device_message )
    {
        openInput( device_message.header_.source_address_, device_message.header_.source_port_ );
        return pullAs<__Payload>();
    }

    template<class __Payload>
    void pullDeviceMessage( InputTCPDeviceMessage<__Payload> & device_message )
    {
        openInput( device_message.header_.source_address_, device_message.header_.source_port_ );
        device_message.header_ = input_state_;
        pull( device_message.payload_ );
    }
};

#endif // _MESSAGES_INPUTTCPDEVICE_H_
