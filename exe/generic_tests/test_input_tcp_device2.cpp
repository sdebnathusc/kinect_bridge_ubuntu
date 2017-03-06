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

int main( int argc, char ** argv )
{
    try
    {
        std::cout << "connecting to server..." << std::endl;
        InputTCPDevice input_tcp_device( "localhost", 5903 );

        std::cout << "connected" << std::endl;

        while( input_tcp_device.input_socket_.impl()->initialized() )
        {
            while( !input_tcp_device.input_socket_.available() )
            {
                std::cout << "waiting for content..." << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            }

            std::cout << "pulling message..." << std::endl;

            CodedMessage<> message;
            // pull in one message
            input_tcp_device.pull( message );

            std::cout << "got message: " << message.header_.encoding_ << " | " << message.header_.payload_id_ << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
        }

    }
    catch( std::exception & e )
    {
        std::cout << e.what() << std::endl;
        std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
    }

/*
    {
        std::ofstream debug_stream( "debug2-out" );
        atomics::BinaryOutputStream binary_stream( debug_stream, atomics::BinaryOutputStream::NETWORK_BYTE_ORDER );

        KinectSpeechMessage speech_message;

        KinectSpeechPhraseMessage speech_phrase_message;

        speech_phrase_message.tag_ = "SOMETAG";
        speech_phrase_message.confidence_ = 0.53;

        speech_message.payload_.push_back( speech_phrase_message );

        MessageCoder<BinaryCodec<> > binary_coder;
        std::cout << "----- encoding speech message -----" << std::endl;
        CodedMessage<> binary_coded_message = binary_coder.encode( speech_message );
        std::cout << "----- packing coded message -----" << std::endl;
        binary_coded_message.pack( binary_stream );

        binary_stream.flush();
        debug_stream.close();
    }
    std::cout << "==========" << std::endl;
*/
/*
    {
        std::ifstream input_stream( "debug2-out" );
        atomics::BinaryInputStream binary_stream( input_stream, atomics::BinaryInputStream::NETWORK_BYTE_ORDER );

        CodedMessage<> binary_coded_message;

        std::cout << "------ unpacking coded message -----" << std::endl;
        binary_coded_message.unpack( binary_stream );

        std::cout << binary_coded_message.header_.encoding_ << std::endl;
        std::cout << binary_coded_message.header_.payload_type_ << std::endl;
        std::cout << binary_coded_message.header_.decoded_size_ << std::endl;
    }
*/
    std::this_thread::sleep_for( std::chrono::seconds( 60 ) );

    return 0;
}
