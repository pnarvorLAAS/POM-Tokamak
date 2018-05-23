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
            TokamakASN1(int32_t freq); 
            TokamakASN1(int32_t freq, int32_t sec); 
            TokamakASN1(int32_t freq, int32_t sec, std::string worldFrame);
            TokamakASN1(int32_t freq, int32_t sec, std::string worldFrame, std::string robotFrame);

            ~TokamakASN1();
            void clean_up();


            // Wrappers of conversion messages
            Pose_InFuse update_publish();
            Pose_InFuse update_respond();

            // Conversion to BitStream
            void encode_publish(BitStream &b);
            void encode_response(BitStream &b);

            // Decoding from ASN1 BitStream
            bool decode_insertPose(BitStream &msg, PositionManager::Pose &poseToDecode);
            bool decode_request(BitStream &msg, PositionManager::Pose &poseToDecode);


    };
};

#endif
