#include <infuse_pom_tokamak/TokamakASN1.hpp>

namespace tokamak{

    TokamakASN1::TokamakASN1(): Tokamak()
    {
        init();
    }

    TokamakASN1::TokamakASN1(std::string worldFrame, std::string robotFrame, std::string graphFrame, std::string baseline): Tokamak(worldFrame,robotFrame, graphFrame, baseline)
    {
        init();
    }

    void TokamakASN1::init()
    {
        poseInFuseInsert    = new asn1SccTransformWithCovariance;
        poseInFuseRespond   = new asn1SccTransformWithCovariance;
        poseInFusePublish   = new asn1SccTransformWithCovariance;
        poseInFuseRequest   = new asn1SccTransformWithCovariance;

        perBuffer = new byte[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING];
        memset(perBuffer,0,asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
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

    asn1SccTransformWithCovariance TokamakASN1::update_publish()
    {
        toASN1SCC(posePublish,*poseInFusePublish);
        return *poseInFusePublish;
    }

    asn1SccTransformWithCovariance TokamakASN1::update_respond()
    {
        toASN1SCC(poseRespond,*poseInFuseRespond);
        return *poseInFuseRespond;
    }

    void TokamakASN1::encode_publish(BitStream &b)
    {
        //Convert the pose to publish from internal to ASN1 C type
        toASN1SCC(posePublish,*poseInFusePublish);

        //Convert the pose to publish from ASN1 C type to ASN1 BitStream
        int errorCode;
        BitStream_Init(&b,perBuffer,asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
        try
        {
            if (!asn1SccTransformWithCovariance_Encode(poseInFusePublish,&b,&errorCode,TRUE))
            {
                throw e_asn1;
            }
        }
        catch (std::exception const & e)
        {
            std::cerr << "[ENCODING FAILED]: " << e.what() << std::endl;
        }
    }
    
    void TokamakASN1::encode_response(BitStream &b)
    {

        //Convert the pose to publish from internal to ASN1 C type
        toASN1SCC(poseRespond,*poseInFuseRespond);

        //Convert the pose to publish from ASN1 C type to ASN1 BitStream
        int errorCode;
        //BitStream_Init(&b,perBuffer,asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
        try
        {
            if (!asn1SccTransformWithCovariance_Encode(poseInFuseRespond,&b,&errorCode,TRUE))
            {
                throw e_asn1;
            }
        }
        catch (std::exception const & e)
        {
            std::cerr << "[ENCODING FAILED]: " << e.what() << std::endl;
        }
    }
    
    bool TokamakASN1::decode_insertPose(BitStream &msg,PositionManager::Pose& poseToDecode)
    {
        int errorCode;
        //std::cout << "Trying to decode insertion pose" << std::endl;
        try 
        {
            if (!asn1SccTransformWithCovariance_Decode(poseInFuseInsert,&msg,&errorCode))
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
        return true;
    }
    
    bool TokamakASN1::decode_request(BitStream &msg, PositionManager::Pose& poseToDecode)
    {
        int errorCode;
        try 
        {
            if (!asn1SccTransformWithCovariance_Decode(poseInFuseRequest,&msg,&errorCode))
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

        return true;
    }

};

