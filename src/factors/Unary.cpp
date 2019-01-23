#include <infuse_pom_tokamak/factors/Unary.hpp>

namespace tokamak
{
    namespace Unary{

        Vector GPSFactor::evaluateError(const Pose3 &site_robot, boost::optional<Matrix&> Hi) const
        {
            //Compose SF->GTF with Robot->SF: Obtain Robot->GTF
            OptionalJacobian<6,6>::Jacobian J_GR_sr;
            const Pose3  global_robot = traits<Pose3>::Compose(global_site_,site_robot,boost::none, J_GR_sr);

            //global_robot.print("GTF-> ROBOT : ");

            //Compute Robot->GTF * Robot->Sensor
            OptionalJacobian<3,6>::Jacobian J_GS_gr;
            const Point3 global_sensor = global_robot.transform_from(robot_sensor_, J_GS_gr, boost::none);

            //global_sensor.print("GTF-> SENSOR: ");

//            estimatedSensorPose.print("\n AIE AIE AIE \n");

            // Update Jacobians
            if (Hi) *Hi = J_GS_gr *  J_GR_sr;


            Point3 error = global_sensor - measure_;
            //error.print("ERROR FOUND:");

            //Return error (Jacobian is I)
            return global_sensor - measure_;

        }
    }
}
