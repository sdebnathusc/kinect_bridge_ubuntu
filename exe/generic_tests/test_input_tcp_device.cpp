#include <messages/output_file_device.h>
#include <messages/input_tcp_device.h>
#include <messages/png_image_message.h>

#include <thread>
#include <chrono>
#include <iostream>

int main( int argc, char ** argv )
{
    try
    {
        std::cout << "connecting to server..." << std::endl;
        InputTCPDevice input_tcp_device( "localhost", 5903 );

        std::cout << "connected; pulling image message..." << std::endl;
        PNGImageMessage<> image_message;
        // pull in one message
        input_tcp_device.pull( image_message );

        std::cout << "pulled message; pushing to file" << std::endl;
        OutputFileDevice output_file_device( "test-out.png", std::ios::binary );

        output_file_device.push( image_message );

        std::cout << "done; exiting in 5 seconds..." << std::endl;

        std::this_thread::sleep_for( std::chrono::seconds( 5 ) );
    }
    catch( std::exception & e )
    {
        std::cout << e.what() << std::endl;
    }

    return 0;
}
