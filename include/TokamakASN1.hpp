#ifndef __TOKAMAK_ASN1_HPP__
#define __TOKAMAK_ASN1_HPP__

#include <Pose_InFuse.h>
#include <Tokamak.hpp>
#include <conversions/asn1_pom_conversions.hpp>

#define e_asn1  Error("Message must be wrong")

namespace tokamak
{
    class TokamakASN1: public Tokamak
    {
        private:
            Pose_InFuse*    poseInFusePublish;
            Pose_InFuse*    poseInFuseRespond;
            Pose_InFuse*    poseInFuseInsert;
            Pose_InFuse*    poseInFuseRequest;
            byte*           perBuffer;

        public:
            TokamakASN1();
            ~TokamakASN1();
            void clean_up();


            // Wrappers of conversion messages
            Pose_InFuse update_publish();
            Pose_InFuse update_respond();

            // Conversion to BitStream
            BitStream encode_publish();
            BitStream encode_response();

            // Decoding from ASN1 BitStream
            void decode_insertPose(BitStream &msg, PositionManager::Pose &poseToDecode);
            void decode_request(BitStream &msg, PositionManager::Pose &poseToDecode);


    };
};

#endif
