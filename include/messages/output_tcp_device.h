#ifndef _MESSAGES_OUTPUTTCPDEVICE_H_
#define _MESSAGES_OUTPUTTCPDEVICE_H_

#include <string>
#include <thread>
#include <memory>

#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketStream.h>

#include <messages/container_messages.h>
#include <messages/binary_message.h>
#include <messages/exceptions.h>

#include <messages/message_coder.h>
#include <messages/binary_codec.h>

// analagous to server; will start up at the given address:port and wait for a client it can stream data to

class OutputTCPDeviceMessageHeader : public RecursiveMessageHeader
{
public:
    std::string destination_address_;
    uint16_t destination_port_;

    OutputTCPDeviceMessageHeader( std::string const & destination_address = "localhost", uint16_t destination_port = 0 )
    :
        destination_address_( destination_address ),
        destination_port_( destination_port )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        RecursiveMessageHeader::pack( archive );
        archive << destination_address_;
        archive << destination_port_;
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        RecursiveMessageHeader::unpack( archive );
        archive >> destination_address_;
        archive >> destination_port_;
    }
};

template<class __Payload = BinaryMessage<> >
class OutputTCPDeviceMessage : public SerializableMessageInterface<OutputTCPDeviceMessageHeader, __Payload>
{
public:
    typedef __Payload _Payload;
    typedef SerializableMessageInterface<OutputTCPDeviceMessageHeader, __Payload> _Message;

    template<class... __Args>
    OutputTCPDeviceMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    DECLARE_MESSAGE_INFO( OutputTCPDeviceMessage )
};

class OutputTCPDevice
{
public:
    typedef Poco::Net::SocketStream _OutputStream;

    bool running_;

    Poco::Net::ServerSocket server_socket_;
    Poco::Net::StreamSocket output_socket_;

    OutputTCPDeviceMessageHeader output_state_;

    std::shared_ptr<std::thread> check_client_thread_ptr_;

    OutputTCPDevice() = default;

    // if we've been passed at least one argument, open the socket and record the state
    template<class __Head, class... __Args>
    OutputTCPDevice( __Head && head, __Args&&... args )
    :
        running_( true ),
        server_socket_( Poco::Net::SocketAddress( std::forward<__Head>( head ), std::forward<__Args>( args )... ) )
    {
        std::cout << "server listening on " << server_socket_.address().toString() << std::endl;
        updateOutputState( std::forward<__Head>( head ), std::forward<__Args>( args )... );
    }

    ~OutputTCPDevice()
    {
        closeOutput();
    }

    void checkClientTask()
    {
        while( running_ )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            if( output_socket_.impl()->initialized() && !checkClientConnection() )
            {
                try
                {
                    output_socket_.shutdown();
                }
                catch( std::exception & e )
                {
                    std::cout << "checkClientTask(): " << e.what() << std::endl;
                }

                output_socket_.close();
                break;
            }
        }
    }

    template<class... __Args>
    void openOutput( __Args&&... args )
    {
        // open the socket and update our state
        server_socket_ = Poco::Net::ServerSocket( Poco::Net::SocketAddress( std::forward<__Args>( args )... ) );
        updateOutputState( std::forward<__Args>( args )... );
    }

    template<class... __Args>
    void updateOutputState( __Args&&... args )
    {
        // record our state
        auto args_tuple = std::make_tuple( std::forward<__Args>( args )... );
        output_state_.destination_address_ = std::get<0>( args_tuple );
        output_state_.destination_port_ = std::get<1>( args_tuple );
    }

    void closeOutput()
    {
        std::cout << "closing output" << std::endl;
        running_ = false;
        try
        {
            // close the socket to the client
            std::cout << "shutting down output socket" << std::endl;
            output_socket_.shutdown();
        }
        catch( std::exception & e )
        {
            std::cout << "closeOutput(): " << e.what() << std::endl;
        }

        std::cout << "closing output socket" << std::endl;
        output_socket_.close();

        // close the server socket
        std::cout << "closing server socket" << std::endl;
        server_socket_.close();

        try
        {
            if( check_client_thread_ptr_ ) check_client_thread_ptr_->join();
        }
        catch( std::exception & e )
        {
            std::cout << "closeOutput(): " << e.what() << std::endl;
        }
    }

    template<class __Serializable>
    void push( __Serializable & serializable )
    {
        // if the server itself hasn't been initialized, then something is wrong; bail
        if( !server_socket_.impl()->initialized() ) throw messages::MessageException( "Failed to serialize message; TCPOutputDevice not initialized" );

        establishClientConnection();

        MessageCoder<BinaryCodec<> > binary_coder;

        auto binary_coded_message = binary_coder.encode( serializable );

        uint32_t message_size = binary_coded_message.payload_.size_;
        char protocol_message[6];
        protocol_message[0] = '<';
        protocol_message[5] = '>';

        *reinterpret_cast<decltype(message_size)*>( protocol_message + 1 ) = message_size;

        try
        {
            // send protocol-layer message
            sendBytes( protocol_message, 6 );
            // send actual message
            sendBytes( binary_coded_message.payload_.data_, message_size );
        }
        catch( messages::MessageException & e )
        {
            std::cout << "failed to send data: " << e.what() << std::endl;
            output_socket_.shutdown();
            output_socket_.close();
        }
    }

    void establishClientConnection()
    {
        // check whether the output socket is ready
        if( !output_socket_.impl()->initialized() )
        {
            // if it isn't, we'll wait for new clients; otherwise we already have a connection to the client
            std::cout << "can't push; waiting for client connection" << std::endl;
            Poco::Net::SocketAddress client_address;
            output_socket_ = server_socket_.acceptConnection( client_address );
            std::cout << "accepted connection from client " << client_address.toString() << std::endl;

            // (re)start client monitoring thread
            if( check_client_thread_ptr_ ) check_client_thread_ptr_->join();
            check_client_thread_ptr_ = std::make_shared<std::thread>( &OutputTCPDevice::checkClientTask, this );
        }
    }

    bool checkClientConnection()
    {
        // check if the client is still connected by attempting a read
        char read_buf;
        try
        {
            if( output_socket_.receiveBytes( &read_buf, 1 ) == 0 )
            {
                std::cout << "client disconnected; can't read" << std::endl;
                return false;
            }
        }
        catch( std::exception & e )
        {
            std::cout << "checkClientConnection(): " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    void sendBytes( char const * bytes, uint32_t length )
    {
        if( !output_socket_.impl()->initialized() ) throw messages::MessageException( "Failed to send data; socket not initialized" );
        uint32_t bytes_sent = 0;
        while( bytes_sent < length )
        {
            int send_result = output_socket_.sendBytes( bytes + bytes_sent, length - bytes_sent );
            if( send_result < 0 ) throw messages::MessageException( "sendBytes() failed" );
            else if( send_result == 0 ) std::cout << "output buffer full" << std::endl;
            else bytes_sent += send_result;
        }
    }

    template<class __Payload>
    void push( OutputTCPDeviceMessage<__Payload> & device_message )
    {
        auto & header = device_message.header_;
        auto & payload = device_message.payload_;

        openOutput( header.destination_address_, header.destination_port_ );

        push( payload );
    }
};

#endif // _MESSAGES_OUTPUTTCPDEVICE_H_
