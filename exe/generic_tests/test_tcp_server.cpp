#include <iostream>
#include <chrono>
#include <thread>

#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketStream.h>

#include <atomics/binary_stream.h>

#include <messages/kinect_messages.h>
#include <messages/binary_codec.h>
#include <messages/message_coder.h>

#include <messages/output_tcp_device.h>

int main()
{
/*
    Poco::Net::SocketAddress socket_address( "localhost", 5903 );
    Poco::Net::ServerSocket server_socket( socket_address );

    for (;;)
    {
        std::cout << "waiting for client..." << std::endl;
        Poco::Net::StreamSocket stream_socket = server_socket.acceptConnection();
        std::cout << "got client; sending data" << std::endl;
        Poco::Net::SocketStream stream( stream_socket );

        std::cout << "preparing data..." << std::endl;

        atomics::BinaryOutputStream binary_stream( stream, atomics::BinaryOutputStream::NETWORK_BYTE_ORDER );

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
        stream.flush();
        stream.close();
        std::cout << "done" << std::endl;
    }
*/

    OutputTCPDevice output_tcp_device( "localhost", 5903 );

    for( ;; )
    {
        KinectSpeechMessage speech_message;

        KinectSpeechPhraseMessage speech_phrase_message;

        speech_phrase_message.tag_ = "SOMETAG";
        speech_phrase_message.confidence_ = 0.53;

        speech_message.payload_.push_back( speech_phrase_message );

        MessageCoder<BinaryCodec<> > binary_coder;
        std::cout << "----- encoding speech message -----" << std::endl;
        CodedMessage<> binary_coded_message = binary_coder.encode( speech_message );
        std::cout << "----- packing coded message -----" << std::endl;
        output_tcp_device.push( binary_coded_message );
    }

    return 0;
}
