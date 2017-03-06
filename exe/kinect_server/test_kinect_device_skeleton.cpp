#include <iostream>
#include <fstream>
#include <thread>

#include <kinect_common/kinect_device.h>

typedef KinectBodiesMessage _BodiesMsg;
typedef std::shared_ptr<_BodiesMsg> _BodiesMsgPtr;

int main()
{
    std::ofstream output_stream( "test_kinect_bodies.pak", std::ios::binary );

    KinectDevice kinect_device;
    while( true )
    {
        try
        {
            kinect_device.initialize( false, false, false, false, true );
            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 250 ) );
            continue;
        }
    }

    _BodiesMsgPtr message_ptr;

    while( true )
    {
        try
        {
            // make sure we're getting data for at least one person
            kinect_device.pullBodies( message_ptr );
            if( message_ptr->size() == 0 )
            {
                std::cout << "no people in view" << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                continue;
            }

            // check to make sure at least one person is tracked; if not, keep looking
            bool is_anyone_tracked = false;
            for( size_t i = 0; i < message_ptr->size(); ++i )
            {
                if( (*message_ptr)[i].is_tracked_ )
                {
                    is_anyone_tracked = true;
                    break;
                }
            }

            if( !is_anyone_tracked )
            {
                std::cout << "no people tracked" << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                continue;
            }

            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
            continue;
        }
    }

    std::cout << "packing raw skeleton message with " << message_ptr->size() << " components" << std::endl;

    // save entire message including timestamp
    message_ptr->pack( output_stream );

    output_stream.close();

    std::cout << "exiting in 3 seconds..." << std::endl;
    std::this_thread::sleep_for( std::chrono::seconds( 3 ) );

    return 0;
}
