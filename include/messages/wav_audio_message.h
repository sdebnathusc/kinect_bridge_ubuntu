#ifndef _MESSAGES_WAVAUDIOMESSAGE_H_
#define _MESSAGES_WAVAUDIOMESSAGE_H_

#include <messages/audio_message.h>
#include <sndfile.h>
#include <sstream>

namespace snd_helper
{
    // read into dest
    template<class __Archive>
    sf_count_t read( void * dest, sf_count_t count, void * data )
    {
//        std::cout << "read: " << count << std::endl;
        __Archive * archive = static_cast<__Archive *>( data );
//        std::cout << "ipos: " << archive->tellp() << std::endl;
        archive->read( reinterpret_cast<char *>( dest ), count );
//        std::cout << "apos: " << archive->tellg() << std::endl;
        return count;
    }

    // write from src
    template<class __Archive>
    sf_count_t write( void const * src, sf_count_t count, void * data )
    {
//        std::cout << "write: " << count << std::endl;
        __Archive * archive = static_cast<__Archive *>( data );
//        std::cout << "ipos: " << archive->tellp() << std::endl;
        archive->write( reinterpret_cast<char const *>( src ), count );
//        std::cout << "apos: " << archive->tellp() << std::endl;
        return count;
    }

    // total file length
    template<class __Archive>
    sf_count_t readFileLen( void * data )
    {
        __Archive * archive = static_cast<__Archive *>( data );

        std::streamsize const last_pos = archive->tellg();

        archive->seekg( 0, std::ios::end );
        sf_count_t length = archive->tellg();

        if( length < 0 )
        {
//            std::cout << "length negative" << std::endl;
            length = 0;
            archive->clear();
            archive->seekg( 0, std::ios::beg );
        }
        else archive->seekg( last_pos, std::ios::beg );

//            std::cout << "snd_helper::read::getFileLen : " << length << std::endl;

        return length;
    }

    // seek to offset relative to whence
    template<class __Archive>
    sf_count_t seekRead( sf_count_t offset, int whence, void * data )
    {
        __Archive * archive = static_cast<__Archive *>( data );
        switch( whence )
        {
        case SEEK_SET:
            archive->seekg( offset );
            break;
        case SEEK_CUR:
            archive->seekg( offset, std::ios::cur );
            break;
        case SEEK_END:
            archive->seekg( offset, std::ios::end );
            break;
        }

        if( archive->tellg() < 0 )
        {
//            std::cout << "tellg negative" << std::endl;
            archive->clear();
            archive->seekg( 0, std::ios::beg );
        }

//            std::cout << "snd_helper::read::seek( " << offset << ", " << whence << " ) : " << archive->tellg() << std::endl;

        return archive->tellg();
    }

    // return current pos
    template<class __Archive>
    sf_count_t tellRead( void * data )
    {
        __Archive * archive = static_cast<__Archive *>( data );
        return archive->tellg();
    }

    // total file length
    template<class __Archive>
    sf_count_t writeFileLen( void * data )
    {
        __Archive * archive = static_cast<__Archive *>( data );

        std::streamsize const last_pos = archive->tellp();

        archive->seekp( 0, std::ios::end );
        sf_count_t length = archive->tellp();
        if( length < 0 )
        {
//            std::cout << "length negative" << std::endl;
            length = 0;
            archive->clear();
            archive->seekp( 0, std::ios::beg );
        }
        else archive->seekp( last_pos, std::ios::beg );

//        std::cout << "snd_helper::write::getFileLen : " << length << " (" << last_pos << ", " << archive->tellp() << ")" << std::endl;

        return length;
    }

    // seek to offset relative to whence
    template<class __Archive>
    sf_count_t seekWrite( sf_count_t offset, int whence, void * data )
    {
        __Archive * archive = static_cast<__Archive *>( data );
        switch( whence )
        {
        case SEEK_SET:
            archive->seekp( offset );
            break;
        case SEEK_CUR:
            archive->seekp( offset, std::ios::cur );
            break;
        case SEEK_END:
            archive->seekp( offset, std::ios::end );
            break;
        }

        if( archive->tellp() < 0 )
        {
//            std::cout << "tellp negative" << std::endl;
            archive->clear();
            archive->seekp( 0, std::ios::beg );
        }

//        std::cout << "snd_helper::write::seek( " << offset << ", " << whence << " ) : " << archive->tellp() << std::endl;

        return archive->tellp();
    }

    // return current pos
    template<class __Archive>
    sf_count_t tellWrite( void * data )
    {
        __Archive * archive = static_cast<__Archive *>( data );
        return archive->tellp();
    }
}

using WAVAudioMessageHeader = AudioMessageHeader<uint32_t, uint16_t, uint32_t>;

template<class __Allocator = std::allocator<char> >
class WAVAudioMessage : public AudioMessage<WAVAudioMessageHeader, __Allocator>
{
public:
    typedef AudioMessage<WAVAudioMessageHeader, __Allocator> _Message;

    template<class... __Args>
    WAVAudioMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    template<class __Archive>
    void pack( __Archive & archive )
    {
        std::stringstream ss;

        SF_VIRTUAL_IO sound_struct;
        sound_struct.get_filelen = &snd_helper::writeFileLen<decltype(ss)>;
        sound_struct.seek = &snd_helper::seekWrite<decltype(ss)>;
        sound_struct.read = &snd_helper::read<decltype(ss)>;
        sound_struct.write = &snd_helper::write<decltype(ss)>;
        sound_struct.tell = &snd_helper::tellWrite<decltype(ss)>;

        SF_INFO info_struct;

        // copy basic info
        info_struct.samplerate = this->header_.sample_rate_;
        info_struct.channels = this->header_.num_channels_;
        info_struct.format = SF_FORMAT_WAV;
        if( this->header_.encoding_ == "PCMS8" ) info_struct.format |= SF_FORMAT_PCM_S8;
        else if( this->header_.encoding_ == "PCMU8" ) info_struct.format |= SF_FORMAT_PCM_U8;
        else if( this->header_.encoding_ == "PCM16" ) info_struct.format |= SF_FORMAT_PCM_16;
        else if( this->header_.encoding_ == "PCM24" ) info_struct.format |= SF_FORMAT_PCM_24;
        else if( this->header_.encoding_ == "PCM32" ) info_struct.format |= SF_FORMAT_PCM_32;
        else if( this->header_.encoding_ == "PCM32F" ) info_struct.format |= SF_FORMAT_FLOAT;

        SNDFILE * sound_handle = sf_open_virtual( &sound_struct, SFM_WRITE, &info_struct, &ss );
//        std::cout << sf_strerror( sound_handle ) << std::endl;
//        runtime_assert_false( sound_handle, 0, "failed to get sound file handle (write) from libsndfile" );

        sf_write_raw( sound_handle, this->payload_.data_, this->payload_.size_ );

        sf_close( sound_handle );

        archive << ss.rdbuf();
    }

    template<class __Archive>
    void unpack( __Archive & archive )
    {
        std::stringstream ss;
        // dump the entire archive to stringstream

//        archive.get( *ss.rdbuf() );
        ss << archive.rdbuf();

//        std::cout << "copied archive to stringstream; size: " << ss.tellp() << std::endl;

        SF_VIRTUAL_IO sound_struct;
        sound_struct.get_filelen = &snd_helper::readFileLen<decltype(ss)>;
        sound_struct.seek = &snd_helper::seekRead<decltype(ss)>;
        sound_struct.read = &snd_helper::read<decltype(ss)>;
        sound_struct.write = &snd_helper::write<decltype(ss)>;
        sound_struct.tell = &snd_helper::tellRead<decltype(ss)>;

        SF_INFO info_struct;
        info_struct.format = 0;

        SNDFILE * sound_handle = sf_open_virtual( &sound_struct, SFM_READ, &info_struct, &ss );
//        std::cout << sf_strerror( sound_handle ) << std::endl;
//        runtime_assert_false( sound_handle, 0, "failed to get sound file handle (read) from libsndfile" );

        // copy basic info
        this->header_.sample_rate_ = info_struct.samplerate;
        this->header_.num_samples_ = info_struct.frames;
        this->header_.num_channels_ = info_struct.channels;

        // calculate sample depth
        uint32_t format_subtype = SF_FORMAT_SUBMASK & info_struct.format;
        switch( format_subtype )
        {
        case SF_FORMAT_PCM_S8:
            this->header_.sample_depth_ = 8;
            this->header_.encoding_ = "PCMS8";
            break;
        case SF_FORMAT_PCM_U8:
            this->header_.sample_depth_ = 8;
            this->header_.encoding_ = "PCMU8";
            break;
        case SF_FORMAT_PCM_16:
            this->header_.sample_depth_ = 16;
            this->header_.encoding_ = "PCM16";
            break;
        case SF_FORMAT_PCM_24:
            this->header_.sample_depth_ = 24;
            this->header_.encoding_ = "PCM24";
            break;
        case SF_FORMAT_PCM_32:
            this->header_.sample_depth_ = 32;
            this->header_.encoding_ = "PCM32";
            break;
        case SF_FORMAT_FLOAT:
            this->header_.sample_depth_ = 32;
            this->header_.encoding_ = "PCM32F";
            break;
        }

        this->payload_.allocate( this->header_.num_samples_ * this->header_.sample_depth_ / 8 );
        sf_read_raw( sound_handle, this->payload_.data_, this->payload_.size_ );

        sf_close( sound_handle );
    }

    DECLARE_MESSAGE_INFO( WAVAudioMessage )
};

#endif // _MESSAGES_WAVAUDIOMESSAGE_H_
