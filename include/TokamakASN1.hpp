#ifndef __TOKAMAK_ASN1_HPP__
#define __TOKAMAK_ASN1_HPP__

#include <Pose_InFuse.h>

namespace tokamak{

    class TokamakASN1: public Tokamak
    {
        private:
            Pose_InFuse*    poseInFuseInput;
            Pose_InFuse*    poseInFuseOutput;
            byte*           perBuffer;

        public:
            TokamakASN1();
            ~TokamaASN1();
            void clean_up();
            
            bool decode_message(BitStream msg);
            bool update_pose(/*poseInFuseInput*/);
            bool update_outputMsg(/*fusedMap,demMsgOutput*/);
            BitStream encode_message(/*demMsgOutput*/);

    };
};

#endif
