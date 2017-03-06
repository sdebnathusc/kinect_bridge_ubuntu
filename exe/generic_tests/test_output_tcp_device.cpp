#include <messages/input_file_device.h>
#include <messages/output_tcp_device.h>
#include <messages/png_image_message.h>

#include <thread>
#include <chrono>
#include <iostream>

int main( int argc, char ** argv )
{
    std::cout << "connecting to file system" << std::endl;
    InputFileDevice input_file_device( "test.png", std::ios::binary );
    PNGImageMessage<> image_message;

    std::cout << "pulling image from file" << std::endl;
    input_file_device.pull( image_message );

    std::cout << "creating output server" << std::endl;
    OutputTCPDevice output_tcp_device( "localhost", 5903 );

    // push out one message then quit
    std::cout << "pushing image out to client" << std::endl;
    output_tcp_device.push( image_message );

    std::cout << "done; exiting in 5 seconds..." << std::endl;

    std::this_thread::sleep_for( std::chrono::seconds( 5 ) );

    return 0;
}
