#ifndef __TOKAMAK_ASN1_HPP__
#define __TOKAMAK_ASN1_HPP__

#include <infuse_asn1_types/TransformWithCovariance.h>
#include <infuse_asn1_conversions/asn1_pom_conversions.hpp>

#include <infuse_pom_tokamak/Tokamak.hpp>

#define e_asn1  Error("Message must be wrong")

namespace tokamak
{
    class TokamakASN1: public Tokamak
    {
        private:
            asn1SccTransformWithCovariance*    poseInFusePublish;
            asn1SccTransformWithCovariance*    poseInFuseRespond;
            asn1SccTransformWithCovariance*    poseInFuseInsert;
            asn1SccTransformWithCovariance*    poseInFuseRequest;
            byte*           perBuffer;

        public:
            TokamakASN1();
            TokamakASN1(std::string worldFrame, std::string robotFrame);
            void init();

            ~TokamakASN1();
            void clean_up();


            // Wrappers of conversion messages
            asn1SccTransformWithCovariance update_publish();
            asn1SccTransformWithCovariance update_respond();

            // Conversion to BitStream
            void encode_publish(BitStream &b);
            void encode_response(BitStream &b);

            // Decoding from ASN1 BitStream
            bool decode_insertPose(BitStream &msg, PositionManager::Pose &poseToDecode);
            bool decode_request(BitStream &msg, PositionManager::Pose &poseToDecode);


    };
};

#endif
