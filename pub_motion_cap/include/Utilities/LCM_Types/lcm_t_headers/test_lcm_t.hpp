/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __lcm_t_headers_test_lcm_t_hpp__
#define __lcm_t_headers_test_lcm_t_hpp__

#include <string>

namespace lcm_t_headers
{

class test_lcm_t
{
    public:
        int32_t    testInteger;
        double     testDouble;
        std::string channelName;
        int8_t     testBoolean;

    public:
        inline int encode(void *buf, int offset, int maxlen) const;
        inline int getEncodedSize() const;
        inline int decode(const void *buf, int offset, int maxlen);
        inline static int64_t getHash();
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int test_lcm_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int test_lcm_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int test_lcm_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t test_lcm_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* test_lcm_t::getTypeName()
{
    return "test_lcm_t";
}

int test_lcm_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->testInteger, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->testDouble, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    char* channelName_cstr = (char*) this->channelName.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &channelName_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->testBoolean, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int test_lcm_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->testInteger, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->testDouble, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    int32_t __channelName_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__channelName_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__channelName_len__ > maxlen - pos) return -1;
    this->channelName.assign(((const char*)buf) + offset + pos, __channelName_len__ - 1);
    pos += __channelName_len__;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->testBoolean, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int test_lcm_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += this->channelName.size() + 4 + 1;
    enc_size += __boolean_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t test_lcm_t::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0x864f70b0757ba2b7LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif