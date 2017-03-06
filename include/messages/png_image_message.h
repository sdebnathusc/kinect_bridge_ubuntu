#ifndef _MESSAGES_PNGIMAGEMESSAGE_H_
#define _MESSAGES_PNGIMAGEMESSAGE_H_

#include <messages/image_message.h>
#include <png.h>
#include <vector>

namespace png_helper
{
    template<class __Archive>
    void readPNG( png_structp png_struct_ptr, png_bytep data, png_size_t size )
    {
//        std::cout << "reading png data size: " << size << " from archive" << std::endl;
        __Archive * archive = static_cast<__Archive *>( png_get_io_ptr( png_struct_ptr ) );
        archive->read( reinterpret_cast<char *>( data ), size );

/*
        for( size_t i = 0; i < size; ++i )
        {
            std::cout << static_cast<uint16_t>( data[i] ) << "|";
        }
        std::cout << std::dec << std::endl;
*/
    }

    template<class __Archive>
    void writePNG( png_structp png_struct_ptr, png_bytep data, png_size_t size )
    {
//        std::cout << "writing png data size: " << size << " to archive" << std::endl;
        __Archive * archive = static_cast<__Archive *>( png_get_io_ptr( png_struct_ptr ) );
        archive->write( reinterpret_cast<char *>( data ), size );

/*
        for( size_t i = 0; i < size; ++i )
        {
            std::cout << static_cast<uint16_t>( data[i] ) << "|";
        }
        std::cout << std::dec << std::endl;
*/
    }
}

template<class __Allocator = std::allocator<char> >
class PNGImageMessage : public ImageMessage<__Allocator>
{
public:
    typedef ImageMessage<__Allocator> _Message;

    uint8_t compression_level_;

    PNGImageMessage()
    :
        _Message(),
        compression_level_( 2 )
    {
        //
    }

    template<class... __Args>
    PNGImageMessage( uint8_t compression_level, __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... ),
        compression_level_( compression_level )
    {
        //
    }

    struct PNGStructAllocator
    {
        enum class StructType
        {
            READ = 0,
            WRITE
        };

        StructType struct_type_;
        png_structp png_struct_ptr_;
        png_infop png_info_ptr_;
        png_infop png_end_ptr_;

//        template<class... __Args>
//        PNGStructAllocator( StructType struct_type, __Args&&... args )
        template<class... __Args>
        PNGStructAllocator( StructType struct_type, __Args&&... args )
        :
            struct_type_( struct_type ),
            png_struct_ptr_( NULL ),
            png_info_ptr_( NULL ),
            png_end_ptr_( NULL )
        {
            switch( struct_type_ )
            {
            case StructType::READ:
                png_struct_ptr_ = png_create_read_struct( std::forward<__Args>( args )... );
//                runtime_assert_false( png_struct_ptr_, 0, "Failed to create read struct" );

                png_info_ptr_ = png_create_info_struct( png_struct_ptr_ );
//                runtime_assert_false( png_info_ptr_, 0, "Failed to create info struct" );

                png_end_ptr_ = png_create_info_struct( png_struct_ptr_ );
//                runtime_assert_false( png_end_ptr_, 0, "Failed to create end struct" );
                break;
            case StructType::WRITE:
                png_struct_ptr_ = png_create_write_struct( std::forward<__Args>( args )... );
//                runtime_assert_false( png_struct_ptr_, 0, "Failed to create write struct" );

                png_info_ptr_ = png_create_info_struct( png_struct_ptr_ );
//                runtime_assert_false( png_info_ptr_, 0, "Failed to create info struct" );

//                runtime_assert_true( png_end_ptr_, 0, "End struct is not null" );
                break;
            }
        }

        ~PNGStructAllocator()
        {
            if( png_struct_ptr_ )
            {
                switch( struct_type_ )
                {
                case StructType::READ:
                    png_destroy_read_struct( &png_struct_ptr_, png_info_ptr_ ? &png_info_ptr_ : NULL, png_end_ptr_ ? &png_end_ptr_ : NULL );
                    break;
                case StructType::WRITE:
                    png_destroy_write_struct( &png_struct_ptr_, png_info_ptr_ ? &png_info_ptr_ : NULL );
                    break;
                }
            }
        }

        png_structp get()
        {
            return png_struct_ptr_;
        }

        png_infop getInfo()
        {
            return png_info_ptr_;
        }

        png_infop getEnd()
        {
            return png_end_ptr_;
        }
    };


/*
    template<class __Archive>
    void readPNG( png_structp png_struct_ptr, png_bytep data, png_size_t size )
    {
        __Archive * archive = static_cast<__Archive *>( png_get_io_ptr( png_struct_ptr ) );
        archive->read( reinterpret_cast<char *>( data ), size );
    }
*/
    // override typical message packing
    // our payload is a raw image (BinaryMessage)
    // by default, we would just dump all the raw image bytes into the given archive
    // instead, we want to first encode the image in PNG format and then write that result to the archive
    // note that we don't actually pack the ImageMessageHeader; this is encoding in the PNG binary and will be extracted during unpacking
    template<class __Archive>
    void pack( __Archive & archive )
    {
//        std::cout << "PNGImageMessage packing into archive" << std::endl;

        PNGStructAllocator png_struct_allocator( PNGStructAllocator::StructType::WRITE, PNG_LIBPNG_VER_STRING, static_cast<png_voidp>( NULL ), static_cast<png_error_ptr>( NULL ), static_cast<png_error_ptr>( NULL ) );

        png_structp png_struct_ptr = png_struct_allocator.get();
        png_infop png_info_ptr = png_struct_allocator.getInfo();

        png_uint_32 color_type;

        auto & header = this->header_;
        auto & payload = this->payload_;

        bool set_filler = false;
        bool set_bgr = false;
        uint8_t num_channels = header.num_channels_;

        if( header.encoding_ == "rgb" || header.encoding_ == "bgr" )
        {
//            std::cout << "packing to RGB" << std::endl;
            color_type = PNG_COLOR_TYPE_RGB;
        }
        if( header.encoding_ == "bgr" || header.encoding_ == "bgra" )
        {
            set_bgr = true;
        }
        if( header.encoding_ == "rgba" || header.encoding_ == "bgra" )
        {
            if( header.num_channels_ == 3 )
            {
//                std::cout << "setting filler; packing to RGB" << std::endl;
                set_filler = true;
                num_channels = 4;

                color_type = PNG_COLOR_TYPE_RGB;
            }
            else
            {
//                std::cout << "packing to RGBA" << std::endl;
                color_type = PNG_COLOR_TYPE_RGB_ALPHA;
            }
        }
        if( header.encoding_ == "gray" ) color_type = PNG_COLOR_TYPE_GRAY;

//        std::cout << "preparing to write PNG: " << header.width_ << "|" << header.height_ << "|" << static_cast<int>( header.pixel_depth_ ) << "|" << static_cast<int>( header.num_channels_ ) << "|" << header.encoding_ << std::endl;

        png_set_write_fn( png_struct_ptr, &archive, &png_helper::writePNG<__Archive>, NULL );
        png_set_IHDR( png_struct_ptr, png_info_ptr, header.width_, header.height_, header.pixel_depth_, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT );
        png_write_info( png_struct_ptr, png_info_ptr );
        png_set_compression_level( png_struct_ptr, compression_level_ );

        if( set_filler ) png_set_filler( png_struct_ptr, 0x00, PNG_FILLER_AFTER );
        if( set_bgr ) png_set_bgr( png_struct_ptr );

        // build pointer pointer map required by png_set_rows
        std::vector<uint8_t *> rows_map( header.height_ );
        for( size_t row = 0; row < rows_map.size(); ++row )
        {
            rows_map[row] = reinterpret_cast<uint8_t *>( payload.data_ ) + row * header.width_ * num_channels * header.pixel_depth_ / 8;
        }

        png_write_image( png_struct_ptr, &rows_map.front() );
        png_write_end( png_struct_ptr, png_info_ptr );

//        png_set_rows( png_struct_ptr, png_info_ptr, &rows_map.front() );
//        png_write_png( png_struct_ptr, png_info_ptr, PNG_TRANSFORM_IDENTITY, NULL );

//        std::cout << "PNGImageMessage done packing into archive" << std::endl;
    }

    // override typical payload unpacking
    // our payload is a raw image (BinaryMessage)
    // by default, we would just pull all the raw image bytes from the given archive
    // instead, we want to first interpret the bytes as PNG, decode to raw, and then dump that result to our decoded payload
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        std::cout << "PNGImageMessage unpacking from archive" << std::endl;

        char png_signature[8];
        archive.read( png_signature, 8 );

/*
        for( size_t i = 0; i < 8; ++i )
        {
            std::cout << static_cast<uint16_t>( png_signature[i] ) << "|";
        }
        std::cout << std::dec << std::endl;
*/

//        runtime_assert_true( png_sig_cmp( reinterpret_cast<png_bytep>( png_signature ), 0, 8 ), 0, "invalid PNG signature" );

        PNGStructAllocator png_struct_allocator( PNGStructAllocator::StructType::READ, PNG_LIBPNG_VER_STRING, (png_voidp)NULL, (png_error_ptr)NULL, (png_error_ptr)NULL );

        png_structp png_struct_ptr = png_struct_allocator.get();
        png_infop png_info_ptr = png_struct_allocator.getInfo();
        png_infop png_end_ptr = png_struct_allocator.getEnd();

        png_set_read_fn( png_struct_ptr, &archive, &png_helper::readPNG<__Archive> );

        png_set_sig_bytes( png_struct_ptr, 8 );
        png_read_info( png_struct_ptr, png_info_ptr );

        this->header_.width_ = png_get_image_width( png_struct_ptr, png_info_ptr );
        this->header_.height_ = png_get_image_height( png_struct_ptr, png_info_ptr );
        this->header_.pixel_depth_ = png_get_bit_depth( png_struct_ptr, png_info_ptr );
        this->header_.num_channels_ = png_get_channels( png_struct_ptr, png_info_ptr );
        uint8_t const color_type = png_get_color_type( png_struct_ptr, png_info_ptr );

        switch( color_type )
        {
        case PNG_COLOR_TYPE_RGB:
            this->header_.encoding_ = "rgb";
            break;
        case PNG_COLOR_TYPE_GRAY:
            this->header_.encoding_ = "gray";
        }

        this->payload_.allocate( this->header_.width_ * this->header_.height_ * this->header_.num_channels_ * this->header_.pixel_depth_ / 8 );

        // build pointer pointer map required by png_set_rows
        std::vector<uint8_t *> rows_map( this->header_.height_ );
        for( size_t row = 0; row < rows_map.size(); ++row )
        {
            rows_map[row] = reinterpret_cast<uint8_t *>( this->payload_.data_ ) + row * this->header_.width_ * this->header_.num_channels_ * this->header_.pixel_depth_ / 8;
        }

        png_read_image( png_struct_ptr, &rows_map.front() );
        png_read_end( png_struct_ptr, png_end_ptr );

        std::cout << "PNGImageMessage done unpacking from archive" << std::endl;
    }

    DECLARE_MESSAGE_INFO( PNGImageMessage )
};

#endif // _MESSAGES_PNGIMAGEMESSAGE_H_
