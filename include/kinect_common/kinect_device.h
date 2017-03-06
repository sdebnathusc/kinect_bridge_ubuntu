#ifndef _KINECTCOMMON_KINECTDEVICE_H_
#define _KINECTCOMMON_KINECTDEVICE_H_

#include <iostream>
#include <locale>
#include <codecvt>

// prevent any includes of Windows.h from including winsock.h
#define _WINSOCKAPI_

// For speech APIs
// NOTE: To ensure that application compiles and links against correct SAPI versions (from Microsoft Speech
//       SDK), VC++ include and library paths should be configured to list appropriate paths within Microsoft
//       Speech SDK installation directory before listing the default system include and library directories,
//       which might contain a version of SAPI that is not appropriate for use together with Kinect sensor.
#include <sapi.h>
__pragma(warning(push))
__pragma(warning(disable:6385 6001)) // Suppress warnings in public SDK header

#include <sphelper.h>
__pragma(warning(pop))

// This is the class ID we expect for the Microsoft Speech recognizer.
// Other values indicate that we're using a version of sapi.h that is
// incompatible with this sample.
//DEFINE_GUID(CLSID_ExpectedRecognizer, 0x495648e7, 0xf7ab, 0x4267, 0x8e, 0x0f, 0xca, 0xfb, 0x7a, 0x33, 0xc1, 0x60);

#include <kinect_common/kinect.h>
#include <kinect_common/memory.h>
#include <kinect_common/exceptions.h>
#include <kinect_common/kinect_audio_stream.h>

#include <messages/kinect_messages.h>

// ####################################################################################################
/*
class KinectDeviceMessageHeader
{
public:
    std::string input_path_;
    std::ios_base::openmode input_mode_;

    KinectDeviceMessageHeader( std::string const & input_path = "", std::ios_base::openmode input_mode = std::ios_base::openmode() )
    :
        input_path_( input_path ),
        input_mode_( input_mode )
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive )
    {
        RecursiveMessageHeader::pack( archive );
        archive << input_path_;
        archive << input_mode_;
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        RecursiveMessageHeader::unpack( archive );
        archive >> input_path_;
        archive >> input_mode_;
    }
};
*/

//template<class __Allocator>
//class KinectColorImageMessage : public MultiMessage<ImageMessage<__Allocator>, TimeStampMessage>

/*
// ####################################################################################################
class KinectFaceMessage : public SerializableInterface
{
public:


    // ====================================================================================================
    KinectFaceMessage()
    :
    {
        //
    }

    // ====================================================================================================
    static std::string const & name()
    {
        static std::string const name( "KinectFaceMessage" );
        return name;
    }

    // ====================================================================================================
    virtual std::string const & vName() const
    {
        return name();
    }
};

// ####################################################################################################
template<class __AudioMessage = AudioMessage<> >
class KinectFacesMessage : public MultiMessage<VectorMessage<KinectFaceMessage>, TimeStampMessage>
{
public:
    typedef MultiMessage<VectorMessage<KinectFaceMessage>, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectFacesMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    std::string const & name() const
    {
        static std::string const name( "KinectFacesMessage" );
        return name;
    }

    // ====================================================================================================
    virtual std::string const & vName() const
    {
        return name();
    }
};
*/

/*
// ####################################################################################################
template<class __Allocator = std::allocator<char> >
class KinectCompressedColorImageMessage : public KinectColorImageMessage<PNGImageMessage<__Allocator> >
{
public:
    typedef KinectColorImageMessage<PNGImageMessage<__Allocator> > _Message;

    typedef typename _Message::_Message _Base;

    // ====================================================================================================
    template<class... __Args>
    KinectCompressedColorImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    std::string const & name() const
    {
        static std::string const name( "KinectCompressedColorImage" );
        return name;
    }

    // ====================================================================================================
    virtual std::string const & vName() const
    {
        return name();
    }
};
*/

    /*
// ####################################################################################################
template<class __Allocator = std::allocator<char> >
class KinectRawColorImageMessage : public KinectColorImageMessage<ImageMessage<__Allocator> >
{
public:
    typedef KinectColorImageMessage<ImageMessage<__Allocator> > _Message;

    typedef typename _Message::_Message _Base;

    // ====================================================================================================
    template<class... __Args>
    KinectRawColorImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    std::string const & name() const
    {
        static std::string const name( "KinectRawColorImageMessage" );
        return name;
    }

    // ====================================================================================================
    virtual std::string const & vName() const
    {
        return name();
    }
};
*/

/*
// ####################################################################################################
template<class __Allocator = std::allocator<char> >
class KinectColorImageMessage : public MultiMessage<ImageMessage<__Allocator>, TimeStampMessage>
{
public:
    typedef MultiMessage<ImageMessage<__Allocator>, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectColorImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    std::string const & name() const
    {
        static std::string const name( "KinectColorImage" );
        return name;
    }

    // ====================================================================================================
    virtual std::string const & vName() const
    {
        return name();
    }
};
*/

/*
// ####################################################################################################
template<class __Allocator>
class KinectDeviceMessage : public SerializableMessageInterface<EmptyMessageHeader, TupleMessage<KinectColorImageMessage<__Allocator> > >
{
public:
    typedef __Payload _Payload;
    typedef SerializableMessageInterface<KinectDeviceMessageHeader, __Payload> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectDeviceMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    std::string const & name() const
    {
        static std::string const name( "KinectDevice" );
        return name;
    }

    // ====================================================================================================
    virtual std::string const & vName() const
    {
        return name();
    }
};
*/

// ####################################################################################################
class KinectDevice
{
public:

    // ####################################################################################################
    struct KinectSensor : public ReleasableWrapper<IKinectSensor>
    {
        // ====================================================================================================
        typedef ReleasableWrapper<IKinectSensor> _ReleasableWrapper;
        template<class... __Args>
        KinectSensor( __Args&&... args )
        :
            _ReleasableWrapper( std::forward<__Args>( args )... )
        {
            //
        }

        // ====================================================================================================
        ~KinectSensor()
        {
            if( _ReleasableWrapper::get() ) _ReleasableWrapper::get()->Close();
        }

        // ====================================================================================================
        void initialize()
        {
            std::cout << "looking for default kinect sensor" << std::endl;
            if( FAILED( GetDefaultKinectSensor( _ReleasableWrapper::getAddr() ) ) ) throw KinectException( "Failed to get default kinect sensor" );
            std::cout << "initializing kinect sensor" << std::endl;
            if( FAILED( _ReleasableWrapper::get()->Open() ) ) throw KinectException( "Failed to open kinect sensor" );
            std::cout << "kinect initialized" << std::endl;
        }
    };

    // ####################################################################################################
    struct ColorFrameReader : public ReleasableWrapper<IColorFrameReader>
    {
        typedef ReleasableWrapper<IColorFrameReader> _ReleasableWrapper;

        // ====================================================================================================
        template<class... __Args>
        ColorFrameReader( __Args&&... args )
        :
            _ReleasableWrapper( std::forward<__Args>( args )... )
        {
            //
        }

        // ====================================================================================================
        void initialize( KinectSensor & kinect_sensor )
        {
            ReleasableWrapper<IColorFrameSource> color_frame_source;
            if( FAILED( kinect_sensor->get_ColorFrameSource( color_frame_source.getAddr() ) ) ) throw KinectException( "Failed to get color frame source" );
            if( FAILED( color_frame_source->OpenReader( _ReleasableWrapper::getAddr() ) ) ) throw KinectException( "Failed to open color frame reader" );
        }
    };

    // ####################################################################################################
    struct DepthFrameReader : public ReleasableWrapper<IDepthFrameReader>
    {
        typedef ReleasableWrapper<IDepthFrameReader> _ReleasableWrapper;

        // ====================================================================================================
        template<class... __Args>
        DepthFrameReader( __Args&&... args )
        :
            _ReleasableWrapper( std::forward<__Args>( args )... )
        {
            //
        }

        // ====================================================================================================
        void initialize( KinectSensor & kinect_sensor )
        {
            ReleasableWrapper<IDepthFrameSource> depth_frame_source;
            if( FAILED( kinect_sensor->get_DepthFrameSource( depth_frame_source.getAddr() ) ) ) throw KinectException( "Failed to get depth frame source" );
            if( FAILED( depth_frame_source->OpenReader( _ReleasableWrapper::getAddr() ) ) ) throw KinectException( "Failed to open depth frame reader" );
        }
    };

    // ####################################################################################################
    struct InfraredFrameReader : public ReleasableWrapper<IInfraredFrameReader>
    {
        typedef ReleasableWrapper<IInfraredFrameReader> _ReleasableWrapper;

        // ====================================================================================================
        template<class... __Args>
        InfraredFrameReader( __Args&&... args )
        :
            _ReleasableWrapper( std::forward<__Args>( args )... )
        {
            //
        }

        // ====================================================================================================
        void initialize( KinectSensor & kinect_sensor )
        {
            ReleasableWrapper<IInfraredFrameSource> infrared_frame_source;
            if( FAILED( kinect_sensor->get_InfraredFrameSource( infrared_frame_source.getAddr() ) ) ) throw KinectException( "Failed to get infrared frame source" );
            if( FAILED( infrared_frame_source->OpenReader( _ReleasableWrapper::getAddr() ) ) ) throw KinectException( "Failed to open infrared frame reader" );
        }
    };

    // ####################################################################################################
    struct AudioBeamFrameReader : public ReleasableWrapper<IAudioBeamFrameReader>
    {
        typedef ReleasableWrapper<IAudioBeamFrameReader> _ReleasableWrapper;

        // ====================================================================================================
        AudioBeamFrameReader()
        {
            //
        }

        // ====================================================================================================
        void initialize( KinectSensor & kinect_sensor )
        {
            ReleasableWrapper<IAudioSource> audio_source;

            if( FAILED( kinect_sensor->get_AudioSource( audio_source.getAddr() ) ) ) throw KinectException( "Failed to get audio source" );
            if( FAILED( audio_source->OpenReader( _ReleasableWrapper::getAddr() ) ) ) throw KinectException( "Failed to open audio beam frame reader" );
        }
    };

    // ####################################################################################################
    struct BodyFrameReader : public ReleasableWrapper<IBodyFrameReader>
    {
        typedef ReleasableWrapper<IBodyFrameReader> _ReleasableWrapper;

        // ====================================================================================================
        BodyFrameReader()
        {
            //
        }

        // ====================================================================================================
        void initialize( KinectSensor & kinect_sensor )
        {
            ReleasableWrapper<IBodyFrameSource> body_frame_source;

            if( FAILED( kinect_sensor->get_BodyFrameSource( body_frame_source.getAddr() ) ) ) throw KinectException( "Failed to get body frame source from kinect sensor" );
            if( FAILED( body_frame_source->OpenReader( _ReleasableWrapper::getAddr() ) ) ) throw KinectException( "Failed to get body frame reader from body frame source" );

        }
    };

    // ####################################################################################################
    struct SpeechRecognizer : public ReleasableWrapper<ISpRecognizer>
    {
        typedef ReleasableWrapper<ISpRecognizer> _ReleasableWrapper;

        // A single audio beam off the Kinect sensor.
        ReleasableWrapper<IAudioBeam> audio_beam_;

        // An IStream derived from the audio beam, used to read audio samples
        ReleasableWrapper<IStream> raw_audio_stream_;

        // Stream for converting 32bit Audio provided by Kinect to 16bit required by speech rec
        std::shared_ptr<KinectAudioStream> kinect_audio_stream_;

        // Stream given to speech recognition engine, which it may convert to some other format for processing
        ReleasableWrapper<ISpStream> speech_stream_;

        // Speech recognizer context
        ReleasableWrapper<ISpRecoContext> speech_context_;

        // Speech grammar
        ReleasableWrapper<ISpRecoGrammar> speech_grammar_;

//        // Event triggered when we detect speech recognition
//        HANDLE speech_event_;

        // ====================================================================================================
        SpeechRecognizer()
        {
            //
        }

        ~SpeechRecognizer()
        {
            if( kinect_audio_stream_.get() ) kinect_audio_stream_->SetSpeechState( false );
            if( _ReleasableWrapper::get() ) _ReleasableWrapper::get()->SetRecoState( SPRST_INACTIVE_WITH_PURGE );

            speech_stream_.release();
            _ReleasableWrapper::release();
            speech_context_.release();
            speech_grammar_.release();

            CoUninitialize();

            kinect_audio_stream_.reset();
            raw_audio_stream_.release();
            audio_beam_.release();
        }

        // ====================================================================================================
        void initialize( KinectSensor & kinect_sensor, std::wstring const & grammar_filename )
        {

            // open audio input
            // ----------------------------------------------------------------------------------------------------
            ReleasableWrapper<IAudioSource> audio_source;
            if( FAILED( kinect_sensor->get_AudioSource( audio_source.getAddr() ) ) ) throw KinectException( "Failed to get audio source for speech recognition" );

            ReleasableWrapper<IAudioBeamList> audio_beam_list;
            if( FAILED( audio_source->get_AudioBeams( audio_beam_list.getAddr() ) ) ) throw KinectException( "Failed to get audio beam list for speech recognition" );

            if( FAILED( audio_beam_list->OpenAudioBeam( 0, audio_beam_.getAddr() ) ) ) throw KinectException( "Failed to open first audio beam for speech recognition" );

            if( FAILED( audio_beam_->OpenInputStream( raw_audio_stream_.getAddr() ) ) ) throw KinectException( "Failed to open an input stream to the first audio beam for speech recognition" );

            kinect_audio_stream_ = std::make_shared<KinectAudioStream>( raw_audio_stream_.get() );

            // create speech recognizer
            // ----------------------------------------------------------------------------------------------------

            // Audio Format for Speech Processing
            WORD AudioFormat = WAVE_FORMAT_PCM;
            WORD AudioChannels = 1;
            DWORD AudioSamplesPerSecond = 16000;
            DWORD AudioAverageBytesPerSecond = 32000;
            WORD AudioBlockAlign = 2;
            WORD AudioBitsPerSample = 16;

            WAVEFORMATEX wfxOut = {AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0};

            if( FAILED( CoInitializeEx( NULL, COINIT_MULTITHREADED ) ) ) throw KinectException( "Failed to initialize COM object for speech stream" );

            if( FAILED( CoCreateInstance( CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpStream), (void**)speech_stream_.getAddr() ) ) ) throw KinectException( "Failed to create speech stream instance" );

            kinect_audio_stream_->SetSpeechState(true);
            if( FAILED( speech_stream_->SetBaseStream( kinect_audio_stream_.get(), SPDFID_WaveFormatEx, &wfxOut ) ) ) throw KinectException( "Failed to create downsampling stream for speech recognition" );

            if( FAILED( CoCreateInstance( CLSID_SpInprocRecognizer, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpRecognizer), (void**)_ReleasableWrapper::getAddr() ) ) ) throw KinectException( "Failed to create speech recognizer instance" );

            _ReleasableWrapper::get()->SetInput( speech_stream_.get(), TRUE );

            ReleasableWrapper<ISpObjectToken> speech_engine_token;
            HRESULT hr = SpFindBestToken( SPCAT_RECOGNIZERS, L"Language=409;Kinect=True", NULL, speech_engine_token.getAddr() );
            if( FAILED( hr ) )
            {
                std::cout << "SpFindBestToken failed with error: " << hr << std::endl;
                throw KinectException( "Failed to load acousic model for Kinect" );
            }

            _ReleasableWrapper::get()->SetRecognizer( speech_engine_token.get() );
            if( FAILED( _ReleasableWrapper::get()->CreateRecoContext( speech_context_.getAddr() ) ) ) throw KinectException( "Failed to create speech recognition context" );

            // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model.
            // This will prevent recognition accuracy from degrading over time.
            if( FAILED( _ReleasableWrapper::get()->SetPropertyNum( L"AdaptationOn", 0 ) ) ) throw KinectException( "Failed to disable speech adaptation" );

            // load grammar
            // ----------------------------------------------------------------------------------------------------
            if( FAILED( speech_context_->CreateGrammar( 1, speech_grammar_.getAddr() ) ) ) throw KinectException( "Failed to create grammar" );

            // Populate recognition grammar from file
            if( FAILED( speech_grammar_->LoadCmdFromFile( grammar_filename.c_str(), SPLO_STATIC ) ) ) throw KinectException( "Failed to load grammar data from file" );

            // start speech rec
            // ----------------------------------------------------------------------------------------------------

            // Specify that all top level rules in grammar are now active
            if( FAILED( speech_grammar_->SetRuleState( NULL, NULL, SPRS_ACTIVE ) ) ) throw KinectException( "Failed to set grammar rule state" );

            // Specify that engine should always be reading audio
            if( FAILED( _ReleasableWrapper::get()->SetRecoState( SPRST_ACTIVE_ALWAYS ) ) ) throw KinectException( "Failed to set speech recognition state" );

            // Specify that we're only interested in receiving recognition events
            if( FAILED( speech_context_->SetInterest( SPFEI(SPEI_RECOGNITION), SPFEI(SPEI_RECOGNITION) ) ) ) throw KinectException( "Failed to subscribe to speech recognition events" );

            // Ensure that engine is recognizing speech and not in paused state
            if( FAILED( speech_context_->Resume( 0 ) ) ) throw KinectException( "Failed to start speech recognition engine" );

//          m_hSpeechEvent = m_pSpeechContext->GetNotifyEventHandle();
        }
    };

/*
    // ####################################################################################################
    struct FaceFrameReader
    {
        std::vector<ReleasableWrapper<IFaceFrameSource> > face_frame_sources_;
        std::vector<ReleasableWrapper<IFaceFrameReader> > face_frame_readers_;

        // ====================================================================================================
        FaceFrameReader()
        :
            face_frame_sources_( BODY_COUNT ),
            face_frame_readers_( BODY_COUNT )
        {
            //
        }

        // ====================================================================================================
        void initialize( KinectSensor & kinect_sensor )
        {
            // create a face frame source + reader to track each body in the fov
            for( size_t i = 0; i < BODY_COUNT; ++i )
            {
                // create the face frame source by specifying the required face frame features
                if( FAILED( CreateFaceFrameSource( kinect_sensor.get(), 0, getDesiredFaceFrameFeatures(), &face_frame_sources_[i].get() ) ) ) throw KinectException( "Failed to create face frame source from kinect sensor" );

                // open the corresponding reader
                if( FAILED( face_frame_sources_[i]->OpenReader( &face_frame_readers_[i].get() ) ) ) throw KinectException( "Failed to open face frame reader from face frame source" );
            }
        }

        static uint16_t const & getDesiredFaceFrameFeatures()
        {
            static uint16_t const desired_face_frame_features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
                    | FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
                    | FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
                    | FaceFrameFeatures::FaceFrameFeatures_Happy
                    | FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
                    | FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
                    | FaceFrameFeatures::FaceFrameFeatures_MouthOpen
                    | FaceFrameFeatures::FaceFrameFeatures_MouthMoved
                    | FaceFrameFeatures::FaceFrameFeatures_LookingAway
                    | FaceFrameFeatures::FaceFrameFeatures_Glasses
                    | FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

            return desired_face_frame_features;
        }
    };
*/

    KinectSensor kinect_sensor_;
    ColorFrameReader color_frame_reader_;
    DepthFrameReader depth_frame_reader_;
    InfraredFrameReader infrared_frame_reader_;
    AudioBeamFrameReader audio_beam_frame_reader_;
    BodyFrameReader body_frame_reader_;
    SpeechRecognizer speech_recognizer_;
//    FaceFrameReader face_frame_reader_;

    bool initialized_;

    // ====================================================================================================
    KinectDevice()
    :
        initialized_( false )
    {
        //
    }

    // ====================================================================================================
    void initialize( bool color = true, bool depth = true, bool infrared = true, bool audio = true, bool body = true, bool speech = true /*, bool face = true*/ )
    {
        if( initialized_ ) return;

        std::cout << "initializing sensor" << std::endl;
        kinect_sensor_.initialize();

        if( color )
        {
            std::cout << "initializing color" << std::endl;
            color_frame_reader_.initialize( kinect_sensor_ );
        }

        if( depth )
        {
            std::cout << "initializing depth" << std::endl;
            depth_frame_reader_.initialize( kinect_sensor_ );
        }

        if( infrared )
        {
            std::cout << "initializing infrared" << std::endl;
            infrared_frame_reader_.initialize( kinect_sensor_ );
        }

        if( audio )
        {
            std::cout << "initializing audio" << std::endl;
            audio_beam_frame_reader_.initialize( kinect_sensor_ );
        }

        if( body /*|| face*/ )
        {
            body_frame_reader_.initialize( kinect_sensor_ );
        }

        if( speech )
        {
//            if( CLSID_ExpectedRecognizer != CLSID_SpInprocRecognizer )
//            {
//                throw KinectException( "Using incorrect version of sapi.h; speech will be disabled" );
//            }
//            else
            {
                speech_recognizer_.initialize( kinect_sensor_, std::wstring( L"grammar.grxml" ) );
            }
        }

/*
        if( face )
        {
            face_frame_reader_.initialize( kinect_sensor_ );
        }
*/

        initialized_ = true;
    }

/*
    // ====================================================================================================
    template<class __Allocator>
    void pullColorImage( KinectColorImageMessage<__Allocator> & image_message )
    {
        ReleasableWrapper<IColorFrame> color_frame;
        ReleasableWrapper<IFrameDescription> color_frame_description;

        if( FAILED( color_frame_reader_->AcquireLatestFrame( color_frame.get() ) ) ) throw KinectException( "Failed to acquire color frame from color frame reader" );
        if( FAILED( color_frame->get_FrameDescription( color_frame_description.get() ) ) ) throw KinectException( "Failed to get description from color frame" );

        int64_t timestamp = 0;
        int width = 0;
        int height = 0;

        auto & payload = image_message.payload_;
        auto & header = image_message.header_;

        if( FAILED( color_frame->get_RelativeTime( &image_message.stamp_ ) ) ) throw KinectException( "Failed to get width from color frame" );
        if( FAILED( color_frame_description->get_Width( &header.width_ ) ) ) throw KinectException( "Failed to get width from color frame" );
        if( FAILED( color_frame_description->get_Height( &header.height_ ) ) ) throw KinectException( "Failed to get height from color frame" );

        ColorImageFormat image_format = ColorImageFormat_None;
        if( FAILED( color_frame->get_RawColorImageFormat( &image_format ) ) ) throw KinectException( "Failed to get image format from color frame" );

        // we only send BGR
        header.num_channels_ = 3;
        header.pixel_depth_ = 8;
        header.encoding_ = "bgr";

        // already in BGR, just need to discard alpha
        if( image_format == ColorImageFormat_Bgra )
        {
            // get image data info
            uint32_t image_size;
            uint32_t * image_data;

            if( FAILED( color_frame->AccessRawUnderlyingBuffer( image_size, reinterpret_cast<BYTE **>( image_data ) ) ) ) throw KinectException( "Failed to get image raw buffer" );

            payload.size_ = header.width_ * header.height_ * header.num_channels_ * header.pixel_depth_ / 8;
            payload_.allocate();

            // copy first 3 values for each pixel
            for( uint32_t input_pixel_idx = 0; uint32_t output_pixel_idx = 0; input_pixel_idx < image_size / 4; ++input_pixel_idx )
            {
                // reinterpret each input pixel (uint32_t) as 4 bytes
                // then use offsets to grab the 3 pixels we care about
                // note that input_pixel_idx is incremented in 4-byte increments
                // and output_pixel_idx is incremented in 3-byte increments
                payload_.data_[output_pixel_idx + 0] = reinterpret_cast<uint8_t *>( image_data + input_pixel_idx )[0];
                payload_.data_[output_pixel_idx + 1] = reinterpret_cast<uint8_t *>( image_data + input_pixel_idx )[1];
                payload_.data_[output_pixel_idx + 2] = reinterpret_cast<uint8_t *>( image_data + input_pixel_idx )[2];
                output_pixel_idx += 3;
            }
        }
        else
        {
            throw KinectException( "Image not in BGRa format" );
        }
    }
*/

    // ====================================================================================================
    template<class __ImageMessage>
    void pullColorImageRGB( KinectColorImageMessage<__ImageMessage> & image_message )
    {
        ReleasableWrapper<IColorFrame> color_frame;
        ReleasableWrapper<IFrameDescription> color_frame_description;

        if( FAILED( color_frame_reader_->AcquireLatestFrame( color_frame.getAddr() ) ) ) throw KinectException( "Failed to acquire color frame from color frame reader" );
        if( FAILED( color_frame->get_FrameDescription( color_frame_description.getAddr() ) ) ) throw KinectException( "Failed to get description from color frame" );

        auto & payload = image_message.payload_;
        auto & header = image_message.header_;

        if( FAILED( color_frame->get_RelativeTime( reinterpret_cast<INT64 *>( &image_message.stamp_ ) ) ) ) throw KinectException( "Failed to get timestamp from color frame" );
        if( FAILED( color_frame_description->get_Width( reinterpret_cast<int *>( &header.width_ ) ) ) ) throw KinectException( "Failed to get width from color frame" );
        if( FAILED( color_frame_description->get_Height( reinterpret_cast<int *>( &header.height_ ) ) ) ) throw KinectException( "Failed to get height from color frame" );

        ColorImageFormat image_format = ColorImageFormat_None;
        if( FAILED( color_frame->get_RawColorImageFormat( &image_format ) ) ) throw KinectException( "Failed to get image format from color frame" );

        header.num_channels_ = 3;
        header.pixel_depth_ = 8;
        header.encoding_ = "rgb";

        uint32_t stride = header.width_ * header.num_channels_ * header.pixel_depth_ / 8;

        // 3 channels for rgb
        payload.allocate( stride * header.height_ );

        if( image_format == ColorImageFormat_Rgba )
        {
            // get image data info
            uint32_t image_size = 0;
            uint32_t * image_data = NULL;

            if( FAILED( color_frame->AccessRawUnderlyingBuffer( &image_size, reinterpret_cast<BYTE **>( &image_data ) ) ) ) throw KinectException( "Failed to get image raw buffer" );

            // copy first 3 values for each pixel
            for( uint32_t input_pixel_idx = 0, output_pixel_idx = 0; input_pixel_idx < image_size / 4; input_pixel_idx ++, output_pixel_idx += 3 )
            {
                // since image_data is a 4-byte value, incrementing it by one actually jumps four values ahead
                // use offsets to grab the 3 pixels we care about
                // note that input_pixel_idx is incremented in 4-byte increments
                // and output_pixel_idx is incremented in 3-byte increments
                payload.data_[output_pixel_idx + 0] = reinterpret_cast<uint8_t *>( image_data + input_pixel_idx )[0];
                payload.data_[output_pixel_idx + 1] = reinterpret_cast<uint8_t *>( image_data + input_pixel_idx )[1];
                payload.data_[output_pixel_idx + 2] = reinterpret_cast<uint8_t *>( image_data + input_pixel_idx )[2];
            }
        }
        else
        {
            BinaryMessage<> rgba_message;
            rgba_message.allocate( header.width_ * header.height_ * 4 );
            color_frame->CopyConvertedFrameDataToArray( rgba_message.size_, reinterpret_cast<BYTE*>( rgba_message.data_ ), ColorImageFormat_Rgba );

            // copy first 3 values for each pixel
            for( uint32_t input_pixel_idx = 0, output_pixel_idx = 0; input_pixel_idx < rgba_message.size_; input_pixel_idx += 4, output_pixel_idx += 3 )
            {
                // use offsets to grab the 3 pixels we care about
                // note that input_pixel_idx is incremented in 4-byte increments
                // and output_pixel_idx is incremented in 3-byte increments

                payload.data_[output_pixel_idx + 0] = rgba_message.data_[input_pixel_idx + 0];
                payload.data_[output_pixel_idx + 1] = rgba_message.data_[input_pixel_idx + 1];
                payload.data_[output_pixel_idx + 2] = rgba_message.data_[input_pixel_idx + 2];
            }
        }
    }

    // ====================================================================================================
    template<class __ImageMessage>
    void pullColorImageRGBA( KinectColorImageMessage<__ImageMessage> & image_message )
    {
        ReleasableWrapper<IColorFrame> color_frame;
        ReleasableWrapper<IFrameDescription> color_frame_description;

        if( FAILED( color_frame_reader_->AcquireLatestFrame( color_frame.getAddr() ) ) ) throw KinectException( "Failed to acquire color frame from color frame reader" );
        if( FAILED( color_frame->get_FrameDescription( color_frame_description.getAddr() ) ) ) throw KinectException( "Failed to get description from color frame" );

        auto & payload = image_message.payload_;
        auto & header = image_message.header_;

        if( FAILED( color_frame->get_RelativeTime( reinterpret_cast<INT64 *>( &image_message.stamp_ ) ) ) ) throw KinectException( "Failed to get timestamp from color frame" );
        if( FAILED( color_frame_description->get_Width( reinterpret_cast<int *>( &header.width_ ) ) ) ) throw KinectException( "Failed to get width from color frame" );
        if( FAILED( color_frame_description->get_Height( reinterpret_cast<int *>( &header.height_ ) ) ) ) throw KinectException( "Failed to get height from color frame" );

        ColorImageFormat image_format = ColorImageFormat_None;
        if( FAILED( color_frame->get_RawColorImageFormat( &image_format ) ) ) throw KinectException( "Failed to get image format from color frame" );

        header.num_channels_ = 4;
        header.pixel_depth_ = 8;
        header.encoding_ = "rgba";

//        std::cout << "allocating payload" << std::endl;
        // 4 channels for rgba
        payload.allocate( header.width_ * header.height_ * 4 );

//        std::cout << "copying into payload" << std::endl;
        if( image_format == ColorImageFormat_Rgba )
        {
            // get image data info
            uint32_t image_size = 0;
            uint32_t * image_data = NULL;

//            std::cout << "getting image raw buffer" << std::endl;
            if( FAILED( color_frame->AccessRawUnderlyingBuffer( &image_size, reinterpret_cast<BYTE **>( &image_data ) ) ) ) throw KinectException( "Failed to get image raw buffer" );

            std::memcpy( payload.data_, image_data, image_size );
        }
        else
        {
//            std::cout << "converting image data from code [" << image_format << "] to RGBa" << std::endl;
            color_frame->CopyConvertedFrameDataToArray( payload.size_, reinterpret_cast<BYTE*>( payload.data_ ), ColorImageFormat_Rgba );
        }
    }

    // ====================================================================================================
    template<class __Message>
    void pullColorImage( std::shared_ptr<__Message> & image_message_ptr, std::string const & format = "RGB" )
    {
        if( !image_message_ptr ) image_message_ptr = std::make_shared<__Message>();

        if( format == "RGB" ) pullColorImageRGB( *image_message_ptr );
        if( format == "RGBA" ) pullColorImageRGBA( *image_message_ptr );

//        return image_message_ptr;
    }

    // ====================================================================================================
    template<class __ImageMessage>
    void pullDepthImage( KinectDepthImageMessage<__ImageMessage> & image_message )
    {
        ReleasableWrapper<IDepthFrame> depth_frame;
        ReleasableWrapper<IFrameDescription> depth_frame_description;

        if( FAILED( depth_frame_reader_->AcquireLatestFrame( depth_frame.getAddr() ) ) ) throw KinectException( "Failed to acquire depth frame from depth frame reader" );
        if( FAILED( depth_frame->get_FrameDescription( depth_frame_description.getAddr() ) ) ) throw KinectException( "Failed to get description from depth frame" );

        auto & payload = image_message.payload_;
        auto & header = image_message.header_;

        if( FAILED( depth_frame->get_RelativeTime( reinterpret_cast<INT64 *>( &image_message.stamp_ ) ) ) ) throw KinectException( "Failed to get timestamp from depth frame" );
        if( FAILED( depth_frame->get_DepthMinReliableDistance( &image_message.min_reliable_distance_ ) ) ) throw KinectException( "Failed to get min reliable depth from depth frame" );
        if( FAILED( depth_frame->get_DepthMaxReliableDistance( &image_message.max_reliable_distance_ ) ) ) throw KinectException( "Failed to get max reliable depth from depth frame" );
        if( FAILED( depth_frame_description->get_Width( reinterpret_cast<int *>( &header.width_ ) ) ) ) throw KinectException( "Failed to get width from depth frame" );
        if( FAILED( depth_frame_description->get_Height( reinterpret_cast<int *>( &header.height_ ) ) ) ) throw KinectException( "Failed to get height from depth frame" );

        header.num_channels_ = 1;
        header.pixel_depth_ = 16;
        header.encoding_ = "gray";

        uint32_t stride = header.width_ * header.num_channels_ * header.pixel_depth_ / 8;

        payload.allocate( stride * header.height_ );

        // get image data info
        // image_size is number of pixels
        uint32_t image_size = 0;
        uint16_t * image_data = NULL;

        if( FAILED( depth_frame->AccessUnderlyingBuffer( &image_size, &image_data ) ) ) throw KinectException( "Failed to get image raw buffer" );

        std::memcpy( payload.data_, image_data, payload.size_ );
    }

    // ====================================================================================================
    template<class __Message>
    void pullDepthImage( std::shared_ptr<__Message> & image_message_ptr )
    {
        if( !image_message_ptr ) image_message_ptr = std::make_shared<__Message>();

        pullDepthImage( *image_message_ptr );
    }

    // ====================================================================================================
    template<class __ImageMessage>
    void pullInfraredImage( KinectInfraredImageMessage<__ImageMessage> & image_message )
    {
        ReleasableWrapper<IInfraredFrame> infrared_frame;
        ReleasableWrapper<IFrameDescription> infrared_frame_description;

        if( FAILED( infrared_frame_reader_->AcquireLatestFrame( infrared_frame.getAddr() ) ) ) throw KinectException( "Failed to acquire infrared frame from infrared frame reader" );
        if( FAILED( infrared_frame->get_FrameDescription( infrared_frame_description.getAddr() ) ) ) throw KinectException( "Failed to get description from infrared frame" );

        auto & payload = image_message.payload_;
        auto & header = image_message.header_;

        if( FAILED( infrared_frame->get_RelativeTime( reinterpret_cast<INT64 *>( &image_message.stamp_ ) ) ) ) throw KinectException( "Failed to get timestamp from infrared frame" );
        if( FAILED( infrared_frame_description->get_Width( reinterpret_cast<int *>( &header.width_ ) ) ) ) throw KinectException( "Failed to get width from infrared frame" );
        if( FAILED( infrared_frame_description->get_Height( reinterpret_cast<int *>( &header.height_ ) ) ) ) throw KinectException( "Failed to get height from infrared frame" );

        header.num_channels_ = 1;
        header.pixel_depth_ = 16;
        header.encoding_ = "gray";

        uint32_t stride = header.width_ * header.num_channels_ * header.pixel_depth_ / 8;

        payload.allocate( stride * header.height_ );

        // get image data info
        // image_size is number of pixels
        uint32_t image_size = 0;
        uint16_t * image_data = NULL;

        if( FAILED( infrared_frame->AccessUnderlyingBuffer( &image_size, &image_data ) ) ) throw KinectException( "Failed to get image raw buffer" );

        std::memcpy( payload.data_, image_data, payload.size_ );
    }

    // ====================================================================================================
    template<class __Message>
    void pullInfraredImage( std::shared_ptr<__Message> & image_message_ptr )
    {
        if( !image_message_ptr ) image_message_ptr = std::make_shared<__Message>();

        pullInfraredImage( *image_message_ptr );
    }

    // ====================================================================================================
    template<class __AudioMessage>
    void pullAudio( KinectAudioMessage<__AudioMessage> & audio_message )
    {
        ReleasableWrapper<IAudioBeamFrameList> audio_beam_frame_list;
        ReleasableWrapper<IAudioBeamFrame> audio_beam_frame;

        if( FAILED( audio_beam_frame_reader_->AcquireLatestBeamFrames( audio_beam_frame_list.getAddr() ) ) ) throw KinectException( "Failed to acquire audio beam frame from audio beam frame reader" );
        if( FAILED( audio_beam_frame_list->OpenAudioBeamFrame( 0, audio_beam_frame.getAddr() ) ) ) throw KinectException( "Failed to open first audio beam frame" );

        uint32_t num_subframes = 0;

        if( FAILED( audio_beam_frame->get_SubFrameCount( &num_subframes ) ) ) throw KinectException( "Failed to get number of subframes in audio beam frame" );

        auto & payload = audio_message.payload_;
        auto & header = audio_message.header_;

        // set by the Kinect SDK
        header.num_channels_ = 1;
        header.sample_depth_ = 8 * sizeof( float );
        header.sample_rate_ = 16000;
        header.encoding_ = "PCM32F";

        // should be 256
        header.num_samples_ = num_subframes * 1024 / ( header.num_channels_ * header.sample_depth_ / 8 );

        // each subframe is 1024 bytes long
        payload.allocate( num_subframes * 1024 );

        for( uint32_t i = 0, payload_offset = 0; i < num_subframes; ++i )
        {
            ReleasableWrapper<IAudioBeamSubFrame> audio_beam_subframe;

            if( FAILED( audio_beam_frame->GetSubFrame( i, audio_beam_subframe.getAddr() ) ) ) throw KinectException( "Failed to get subframe from audio beam frame" );

            uint32_t subframe_size = 0;
            float * subframe_data = NULL;

            if( FAILED( audio_beam_subframe->AccessUnderlyingBuffer( &subframe_size, reinterpret_cast<BYTE **>( &subframe_data ) ) ) ) throw KinectException( "Failed to get subframe raw buffer" );

            std::memcpy( payload.data_ + payload_offset, subframe_data, subframe_size );
            payload_offset += subframe_size;

            float beam_angle = 0;
            float beam_angle_confidence = 0;

            if( FAILED( audio_beam_subframe->get_BeamAngle( &beam_angle ) ) ) throw KinectException( "Failed to get beam angle from audio beam subframe" );
            if( FAILED( audio_beam_subframe->get_BeamAngleConfidence( &beam_angle_confidence ) ) ) throw KinectException( "Failed to get beam angle confidence from audio beam subframe" );

            audio_message.beam_angle_ += beam_angle;
            audio_message.beam_angle_confidence_ += beam_angle_confidence;
        }

        audio_message.beam_angle_ /= num_subframes;
        audio_message.beam_angle_confidence_ /= num_subframes;
    }

    // ====================================================================================================
    template<class __Message>
    void pullAudio( std::shared_ptr<__Message> & audio_message_ptr )
    {
        if( !audio_message_ptr ) audio_message_ptr = std::make_shared<__Message>();

        pullAudio( *audio_message_ptr );
    }

    // ====================================================================================================
    void pullBodies( KinectBodiesMessage & bodies_message )
    {
        std::array<ReleasableWrapper<IBody>, BODY_COUNT> bodies;
        ReleasableWrapper<IBodyFrame> body_frame;

        // address map to pass to IBodyFrame::GetAndRefreshBodyData
        std::array<IBody*, BODY_COUNT> bodies_array = { NULL };
//        bodies_array.fill( NULL );

        // pull body data
        if( FAILED( body_frame_reader_->AcquireLatestFrame( body_frame.getAddr() ) ) ) throw KinectException( "Failed to acquire body frame" );

        if( FAILED( body_frame->GetAndRefreshBodyData( BODY_COUNT, bodies_array.data() ) ) ) throw KinectException( "Failed to get body data from body frame" );
        else
        {
            // fill out addresses
            for( size_t i = 0; i < bodies.size(); ++i )
            {
                bodies[i].get() = bodies_array[i];
            }
        }

        // fill out bodies message
        if( FAILED( body_frame->get_RelativeTime( reinterpret_cast<INT64 *>( &bodies_message.stamp_ ) ) ) ) throw KinectException( "Failed to get timestamp from body frame" );

        auto & header = bodies_message.header_;
        auto & payload = bodies_message.payload_;

//        if( bodies_message.size() < BODY_COUNT ) bodies_message.resize( BODY_COUNT );

        payload.clear();

        for( size_t bodies_idx = 0; bodies_idx < bodies.size(); ++bodies_idx )
        {
            if( !bodies[bodies_idx].get() ) continue;

            auto & body = bodies[bodies_idx];
            KinectBodyMessage body_msg;
//            auto & body_msg = bodies_message[bodies_idx];

            if( FAILED( body->get_IsTracked( &body_msg.is_tracked_ ) ) ) throw KinectException( "Failed to get body tracking state" );
            if( FAILED( body->get_HandLeftState( reinterpret_cast<HandState *>( &body_msg.hand_state_left_ ) ) ) ) throw KinectException( "Failed to get left hand state" );
            if( FAILED( body->get_HandRightState( reinterpret_cast<HandState *>( &body_msg.hand_state_right_ ) ) ) ) throw KinectException( "Failed to get right hand state" );
            if( FAILED( body->get_TrackingId( &body_msg.tracking_id_ ) ) ) throw KinectException( "Failed to get body tracking ID" );

            // only fill out joint info if body is tracked
            if( body_msg.is_tracked_ )
            {
                std::array<Joint, JointType_Count> joint_points;
                std::array<JointOrientation, JointType_Count> joint_orientations;

                auto & joints_msg = body_msg.joints_;

                if( joints_msg.size() < JointType_Count ) joints_msg.resize( JointType_Count );
                body->GetJoints( joint_points.size(), joint_points.data() );
                body->GetJointOrientations( joint_orientations.size(), joint_orientations.data() );

                for( size_t joints_idx = 0; joints_idx < joint_points.size(); ++joints_idx )
                {
                    auto & joint_point = joint_points[joints_idx];
                    auto & joint_orientation = joint_orientations[joints_idx];
                    auto & joint_msg = joints_msg[joints_idx];

                    joint_msg.position_.x = joint_point.Position.X;
                    joint_msg.position_.y = joint_point.Position.Y;
                    joint_msg.position_.z = joint_point.Position.Z;

                    joint_msg.orientation_.x = joint_orientation.Orientation.x;
                    joint_msg.orientation_.y = joint_orientation.Orientation.y;
                    joint_msg.orientation_.z = joint_orientation.Orientation.z;
                    joint_msg.orientation_.w = joint_orientation.Orientation.w;

                    joint_msg.joint_type_ = static_cast<KinectJointMessage::JointType>( joint_point.JointType );
                    joint_msg.tracking_state_ = static_cast<KinectJointMessage::TrackingState>( joint_point.TrackingState );
                }
            }

            payload.emplace_back( std::move( body_msg ) );
        }
    }

    // ====================================================================================================
    template<class __Message>
    void pullBodies( std::shared_ptr<__Message> & bodies_message_ptr )
    {
        if( !bodies_message_ptr ) bodies_message_ptr = std::make_shared<__Message>();

        pullBodies( *bodies_message_ptr );
    }

    // ====================================================================================================
    void pullSpeech( KinectSpeechMessage & speech_message )
    {
        auto & header = speech_message.header_;
        auto & payload = speech_message.payload_;
        payload.clear();

        SPEVENT speech_event = {SPEI_UNDEFINED, SPET_LPARAM_IS_UNDEFINED, 0, 0, 0, 0};
        ULONG num_events_fetched = 0;

        // look for a single event
        speech_recognizer_.speech_context_->GetEvents( 1, &speech_event, &num_events_fetched );

        if( num_events_fetched > 0 )
        {

            switch( speech_event.eEventId )
            {
            case SPEI_RECOGNITION:
                if( SPET_LPARAM_IS_OBJECT == speech_event.elParamType )
                {
                    ISpRecoResult* recognition_result = reinterpret_cast<ISpRecoResult*>( speech_event.lParam );
                    SPPHRASE* speech_phrase = NULL;

                    if( FAILED( recognition_result->GetPhrase( &speech_phrase ) ) ) throw KinectException( "Failed to get phrase from speech recognition result" );

                    speech_message.stamp_ = speech_phrase->ftStartTime;

                    if( speech_phrase->pProperties != NULL && speech_phrase->pProperties->pFirstChild != NULL )
                    {
                        const SPPHRASEPROPERTY* speech_tag = speech_phrase->pProperties->pFirstChild;

                        KinectSpeechPhraseMessage speech_phrase_message;
                        speech_phrase_message.tag_ = std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t>().to_bytes( speech_tag->pszValue );
                        speech_phrase_message.confidence_ = speech_tag->SREngineConfidence;

                        payload.emplace_back( std::move( speech_phrase_message ) );
                    }
                    ::CoTaskMemFree( speech_phrase );
                }
                break;
            }
        }
    }

    // ====================================================================================================
    template<class __Message>
    void pullSpeech( std::shared_ptr<__Message> & speech_message_ptr )
    {
        if( !speech_message_ptr ) speech_message_ptr = std::make_shared<__Message>();

        pullSpeech( *speech_message_ptr );
    }

/*
    // ====================================================================================================
    template<class __Message>
    void pullFaces( KinectFaceMessage<__Message> & bodies_message )
    {
        ReleasableWrapper<IBodyFrame> body_frame;

        face_frame_reader_->AcquireLatestFrame(&pBodyFrame);
        hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
                                                            }
        ReleasableWrapper<IFaceFrame> face_frame;
        ReleasableWrapper<IFrameDescription> face_frame_description;

        if( FAILED( face_frame_reader_->AcquireLatestFrame( &face_frame.get() ) ) ) throw KinectException( "Failed to acquire face frame from face frame reader" );
        if( FAILED( face_frame->get_FrameDescription( &face_frame_description.get() ) ) ) throw KinectException( "Failed to get description from face frame" );

        auto & payload = bodies_message.payload_;
        auto & header = bodies_message.header_;

        if( FAILED( face_frame->get_RelativeTime( reinterpret_cast<INT64 *>( &bodies_message.stamp_ ) ) ) ) throw KinectException( "Failed to get timestamp from face frame" );
        if( FAILED( face_frame_description->get_Width( reinterpret_cast<int *>( &header.width_ ) ) ) ) throw KinectException( "Failed to get width from face frame" );
        if( FAILED( face_frame_description->get_Height( reinterpret_cast<int *>( &header.height_ ) ) ) ) throw KinectException( "Failed to get height from face frame" );

        header.num_channels_ = 1;
        header.pixel_depth_ = 16;
        header.encoding_ = "gray";

        uint32_t stride = header.width_ * header.num_channels_ * header.pixel_depth_ / 8;

        payload.allocate( stride * header.height_ );

        // get  data info
        // _size is number of pixels
        uint32_t _size = 0;
        uint16_t * _data = NULL;

        if( FAILED( face_frame->AccessUnderlyingBuffer( &_size, &_data ) ) ) throw KinectException( "Failed to get  raw buffer" );

        std::memcpy( payload.data_, _data, payload.size_ );
    }

    // ====================================================================================================
    template<class __Message>
    void pullFaces( std::shared_ptr<__Message> & faces_message_ptr )
    {
        if( !faces_message_ptr ) faces_message_ptr = std::make_shared<__Message>();

        pullFaces( *face_message_ptr );
    }
*/

    /*
    // ====================================================================================================
    template<class __Serializable>
    __Serializable pullAs()
    {
        __Serializable serializable;
        pull( serializable );
        return serializable;
    }

    // ====================================================================================================
    template<class __Payload>
    KinectDeviceMessage<__Payload> pullDeviceMessage()
    {
        return KinectDeviceMessage<__Payload>( KinectDeviceMessageHeader( input_state_ ), pull<__Payload>() );
    }

    // ====================================================================================================
    template<class __Payload>
    __Payload pullFrom( KinectDeviceMessage<__Payload> const & device_message )
    {
        initialize();
        return pullAs<__Payload>();
    }

    // ====================================================================================================
    template<class __Payload>
    void pullDeviceMessage( KinectDeviceMessage<__Payload> & device_message )
    {
        initialize();
        pull( device_message.payload_ );
    }
    */
};

/*
        // set by the Kinect SDK
        header.num_channels_ = 1;
        header.sample_depth_ = 8 * sizeof( float );
        header.sample_rate_ = 16000;
        header.encoding_ = "PCM32";

        std::cout << "fetching " << num_subframes << " subframes" << std::endl;

        std::vector<std::shared_ptr<BinaryMessage<> > > audio_beam_subframes( num_subframes );

        uint32_t frame_size = 0;

        for( uint32_t i = 0, payload_offset = 0; i < num_subframes; ++i )
        {
            ReleasableWrapper<IAudioBeamSubFrame> audio_beam_subframe;

            if( FAILED( audio_beam_frame->GetSubFrame( i, &audio_beam_subframe.get() ) ) ) throw KinectException( "Failed to get subframe from audio beam frame" );

            uint32_t subframe_size = 0;
            float * subframe_data = NULL;

            if( FAILED( audio_beam_subframe->AccessUnderlyingBuffer( &subframe_size, reinterpret_cast<BYTE **>( &subframe_data ) ) ) ) throw KinectException( "Failed to get subframe raw buffer" );


            auto & binary_message_ptr = audio_beam_subframes.at( i )
            binary_message_ptr = std::make_shared<BinaryMessage<> >();
            binary_message_ptr->allocate( subframe_size );

            std::memcpy( binary_message->data_, subframe_data, subframe_size );

            std::cout << "fetched subframe size: " << subframe_size << std::endl;
        }

        header.num_samples_ = frame_size / ( header.num_channels_ * header.sample_depth_ / 8 );

        payload.allocate( frame_size );

        for( uint32_t i = 0, payload_offset = 0; i < audio_beam_subframes.size(); ++i )
        {
            auto & audio_beam_subframe = *audio_beam_subframes.at( i );

            std::memcpy( payload.data_ + payload_offset, subframe_data, subframe_size );
            payload_offset += subframe_size;
        }
*/

#endif // _KINECTCOMMON_KINECTDEVICE_H_
