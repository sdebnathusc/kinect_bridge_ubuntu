#include <iostream>
#include <fstream>
#include <thread>

#include <kinect_common/kinect_device.h>

typedef KinectSpeechMessage _SpeechMsg;
typedef std::shared_ptr<_SpeechMsg> _SpeechMsgPtr;

int main()
{
    std::ofstream output_stream( "test_kinect_speech.pak", std::ios::binary );

    KinectDevice kinect_device;
    while( true )
    {
        try
        {
            kinect_device.initialize( false, false, false, false, false, true );
            break;
        }
        catch( KinectException & e )
        {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 250 ) );
            continue;
        }
    }

    _SpeechMsgPtr message_ptr;

    while( true )
    {
        try
        {
            kinect_device.pullSpeech( message_ptr );

            if( message_ptr->size() == 0 )
            {
                std::cout << "no speech events received" << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                continue;
            }
            else
            {
                std::cout << "received " << message_ptr->size() << " speech events" << std::endl;
            }

            // check to make sure a phrase has been said with some minimum confidence
            bool is_anything_recognized = false;
            for( size_t i = 0; i < message_ptr->size(); ++i )
            {
                std::cout << (*message_ptr)[i].confidence_ << " | " << (*message_ptr)[i].tag_ << std::endl;
                if( (*message_ptr)[i].confidence_ > 0.3 )
                {
                    is_anything_recognized = true;
                    break;
                }
            }

            if( !is_anything_recognized )
            {
                std::cout << "nothing was recognized" << std::endl;
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

    std::cout << "packing speech message" << std::endl;

    // save entire message including timestamp
    message_ptr->pack( output_stream );

    output_stream.close();

    std::cout << "exiting in 3 seconds..." << std::endl;
    std::this_thread::sleep_for( std::chrono::seconds( 3 ) );

    return 0;
}
