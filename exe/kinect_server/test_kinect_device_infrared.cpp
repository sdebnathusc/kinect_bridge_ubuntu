#include <iostream>
#include <fstream>
#include <thread>

#include <kinect_common/kinect_device.h>
#include <messages/png_image_message.h>

// will accept any message type derived from ImageMessage
// we use PNGImageMessage to allow for PNG compression when packing
// using ImageMessage<> instead would not allow for compression without copying/casting
typedef KinectInfraredImageMessage<PNGImageMessage<> > _ImageMsg;

int main()
{
    std::ofstream output_stream1( "test_kinect_ir.bin", std::ios::binary );
    std::ofstream output_stream2( "test_kinect_ir.png", std::ios::binary );
    std::ofstream output_stream3( "test_kinect_ir.pak", std::ios::binary );

    KinectDevice kinect_device;
    while( true )
    {
        try
        {
            kinect_device.initialize();
            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 250 ) );
            continue;
        }
    }

    std::shared_ptr<_ImageMsg> output_message_ptr;

    while( true )
    {
        try
        {
            kinect_device.pullInfraredImage( output_message_ptr );
            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
            continue;
        }
    }

    std::cout << "packing raw image" << std::endl;

    if( !output_message_ptr ) std::cout << "output message not defined" << std::endl;

    // first, write raw image (3-channel RGB)
    output_message_ptr->packAs<ImageMessage<> >( output_stream1 );

    std::cout << "packing png image" << std::endl;

    // now, encode as png
    output_message_ptr->packAs<PNGImageMessage<> >( output_stream2 );

    std::cout << "packing kinect image" << std::endl;

    // save entire message including timestamp
    output_message_ptr->pack( output_stream3 );

    output_stream1.close();
    output_stream2.close();
    output_stream3.close();

    return 0;
}
