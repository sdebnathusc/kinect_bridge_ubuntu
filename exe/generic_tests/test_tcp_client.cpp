#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>

#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketStream.h>

#include <atomics/binary_stream.h>

#include <messages/kinect_messages.h>
#include <messages/binary_codec.h>
#include <messages/message_coder.h>

#include <messages/input_tcp_device.h>

int main()
{
    try
    {
/*
        Poco::Net::SocketAddress socket_address( "localhost", 5903 );
        Poco::Net::StreamSocket socket( socket_address );
        Poco::Net::SocketStream socket_stream( socket );

        while( !socket_stream )
        {
            std::cout << "waiting for server..." << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
        }

        atomics::BinaryInputStream binary_stream( socket_stream, atomics::BinaryInputStream::NETWORK_BYTE_ORDER );

        CodedMessage<> binary_coded_message;

        std::cout << "------ unpacking coded message -----" << std::endl;
        binary_coded_message.unpack( binary_stream );
*/

        CodedMessage<> binary_coded_message;
        InputTCPDevice input_tcp_device( "localhost", 5903 );
        input_tcp_device.pull( binary_coded_message );

        std::cout << binary_coded_message.header_.encoding_ << std::endl;
        std::cout << binary_coded_message.header_.payload_id_ << std::endl;
        std::cout << binary_coded_message.header_.decoded_size_ << std::endl;

        std::this_thread::sleep_for( std::chrono::seconds( 30 ) );
    }
    catch( std::exception & e )
    {
        std::cout << e.what() << std::endl;
    }

    return 0;
}
