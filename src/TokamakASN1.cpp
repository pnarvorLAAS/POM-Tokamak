#include <TokamakASN1.hpp>

namespace tokamak{

    TokamakASN1::TokamakASN1()
    {
        poseInFuseInsert    = new Pose_InFuse;
        poseInFuseRespond   = new Pose_InFuse;
        poseInFusePublish   = new Pose_InFuse;
        poseInFuseRequest   = new Pose_InFuse;

        perBuffer = new byte[Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING];
        memset(perBuffer,0,Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING);
    }

    TokamakASN1::~TokamakASN1()
    {
        clean_up();
    }

    void TokamakASN1::clean_up()
    {
        std::cout << "Cleaning up Tokamak ASN1" << std::endl;
        delete perBuffer;
        delete poseInFuseInsert;
        delete poseInFuseRespond;
        delete poseInFusePublish;
        delete poseInFuseRequest;
    }

    Pose_InFuse TokamakASN1::update_publish()
    {
        toASN1SCC(posePublish,*poseInFusePublish);
        return *poseInFusePublish;
    }

    Pose_InFuse TokamakASN1::update_respond()
    {
        toASN1SCC(poseRespond,*poseInFuseRespond);
        return *poseInFuseRespond;
    }

    BitStream TokamakASN1::encode_publish()
    {
        //Convert the pose to publish from internal to ASN1 C type
        toASN1SCC(posePublish,*poseInFusePublish);

        //Convert the pose to publish from ASN1 C type to ASN1 BitStream
        int errorCode;
        BitStream b;
        BitStream_Init(&b,perBuffer,Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING);
        try
        {
            if (!Pose_InFuse_Encode(poseInFusePublish,&b,&errorCode,TRUE))
            {
                throw e_asn1;
            }
        }
        catch (std::exception const & e)
        {
            std::cerr << "[ENCODING FAILED]: " << e.what() << std::endl;
        }
        return b;
    }
    
    BitStream TokamakASN1::encode_response()
    {
        //Convert the pose to publish from internal to ASN1 C type
        toASN1SCC(poseRespond,*poseInFuseRespond);

        //Convert the pose to publish from ASN1 C type to ASN1 BitStream
        int errorCode;
        BitStream b;
        BitStream_Init(&b,perBuffer,Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING);
        try
        {
            if (!Pose_InFuse_Encode(poseInFuseRespond,&b,&errorCode,TRUE))
            {
                throw e_asn1;
            }
        }
        catch (std::exception const & e)
        {
            std::cerr << "[ENCODING FAILED]: " << e.what() << std::endl;
        }
        return b;
    }
    
    void TokamakASN1::decode_insertPose(BitStream &msg,PositionManager::Pose& poseToDecode)
    {
        int errorCode;
        try 
        {
            if (!Pose_InFuse_Decode(poseInFuseInsert,&msg,&errorCode))
            {
                throw e_asn1;
            }

            // Transform to Internal format

            fromASN1SCC(*poseInFuseInsert,poseToDecode);
        }
        catch (std::exception const& e)
        {
            std::cerr << "[DECODING FAILED]: " << e.what() << std::endl;
        }
    }
    
    void TokamakASN1::decode_request(BitStream &msg, PositionManager::Pose& poseToDecode)
    {
        int errorCode;
        try 
        {
            if (!Pose_InFuse_Decode(poseInFuseRequest,&msg,&errorCode))
            {
                throw e_asn1;
            }

            // Transform to Internal format

            fromASN1SCC(*poseInFuseRequest,poseToDecode);
        }
        catch (std::exception const& e)
        {
            std::cerr << "[DECODING FAILED]: " << e.what() << std::endl;
        }
    }

};

