#ifndef _ATOMICS_BINARYSTREAM_H_
#define _ATOMICS_BINARYSTREAM_H_

#include <Poco/BinaryWriter.h>
#include <Poco/BinaryReader.h>

namespace atomics
{

class BinaryOutputStream : public Poco::BinaryWriter
{
public:
    typedef Poco::BinaryWriter _BinaryWriter;

    template<class... __Args>
    BinaryOutputStream( __Args&&... args )
    :
        _BinaryWriter( std::forward<__Args>( args )... )
    {
        //
    }

    template<class __Data>
    void write( __Data * data, size_t size )
    {
        return _BinaryWriter::writeRaw( reinterpret_cast<char const *>( data ), size );
    }

//    template<class __Data>
//    BinaryOutputStream & operator<<( __Data const & data ){ *static_cast<_BinaryWriter *>( this ) << data; return *this; }
};

class BinaryInputStream : public Poco::BinaryReader
{
public:
    typedef Poco::BinaryReader _BinaryReader;

    template<class... __Args>
    BinaryInputStream( __Args&&... args )
    :
        _BinaryReader( std::forward<__Args>( args )... )
    {
        //
    }

    template<class __Data>
    void read( __Data * data, size_t size )
    {
        return _BinaryReader::readRaw( reinterpret_cast<char *>( data ), size );
    }

//    template<class __Data>
//    BinaryInputStream & operator>>( __Data & data ){ *static_cast<_BinaryReader *>( this ) >> data; return *this; }
};

} // atomics

#endif // _ATOMICS_BINARYSTREAM_H_
