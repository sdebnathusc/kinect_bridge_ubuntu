#ifndef _MESSAGES_MACROS_H_
#define _MESSAGES_MACROS_H_

#define DECLARE_MESSAGE_ID() \
    static uint32_t ID() \
    { \
        Poco::MD5Engine md5_engine; \
        md5_engine.update( name() ); \
        Poco::DigestEngine::Digest const digest = md5_engine.digest(); \
     \
        /* note that we only use the first 4 of 16 bytes of the digest here */ \
        return *reinterpret_cast<uint32_t const *>( digest.data() ); \
    }

#define DECLARE_MESSAGE_VNAME() \
    virtual std::string const & vName() const \
    { \
        return name(); \
    }

#define DECLARE_MESSAGE_NAME( message_name ) \
static std::string const & name() \
    { \
        static std::string const name( #message_name ); \
        return name; \
    }

#define DECLARE_MESSAGE_INFO( message_name ) \
    DECLARE_MESSAGE_NAME( message_name ) \
    DECLARE_MESSAGE_ID() \
    DECLARE_MESSAGE_VNAME()

#endif // _MESSAGES_MACROS_H_
